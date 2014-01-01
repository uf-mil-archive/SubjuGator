#include <boost/foreach.hpp>

#include <ros/ros.h>
#include <unsupported/Eigen/MatrixFunctions>

#include <odom_estimator/unscented_transform.h>

template<typename Derived>
void assert_none_nan(const MatrixBase<Derived> &m) {
  for(unsigned int i = 0; i < m.rows(); i++) {
    for(unsigned int j = 0; j < m.cols(); j++) {
      assert(std::isfinite(m(i, j)));
    }
  }
}

struct State {
  ros::Time t;
  
  static const unsigned int CURRENT_OFFSET = 0; double current;
  static const unsigned int ANGULAR_VELOCITY_OFFSET = 1; double angular_velocity;
  static const unsigned int EXTERNAL_TORQUE_OFFSET = 2; double external_torque;
  
  static const unsigned int R_OFFSET = 3; double R;
  static const unsigned int L_OFFSET = 4; double L;
  static const unsigned int M_OFFSET = 5; double M;
  static const unsigned int I_OFFSET = 6; double I;
  
  static const unsigned int DELTA_SIZE = 7;
  typedef Matrix<double, DELTA_SIZE, 1> DeltaType;
  
  State(ros::Time t, double current, double angular_velocity, double external_torque, double R, double L,
      double M, double I) :
    t(t), current(current), angular_velocity(angular_velocity),
    external_torque(external_torque), R(R), L(L), M(M), I(I) {
    assert(std::isfinite(current));
    assert(std::isfinite(angular_velocity));
    assert(std::isfinite(external_torque));
    assert(std::isfinite(R));
    assert(std::isfinite(L));
    assert(std::isfinite(M));
    assert(std::isfinite(I));
  }
  
  double dcurrent_over_dt(double voltage) const {
    return (voltage - R * current - M * angular_velocity) / L;
  }
  
  static const unsigned int PREDICT_EXTRA_NOISE_LENGTH = 1;
  Matrix<double, PREDICT_EXTRA_NOISE_LENGTH, PREDICT_EXTRA_NOISE_LENGTH> get_extra_noise_cov() const {
    return scalar_matrix(100);
  }
  State predict(ros::Time t, double voltage,
      Matrix<double, PREDICT_EXTRA_NOISE_LENGTH, 1> noise) const {
    double dt = (t - this->t).toSec();
    
    // d(current)/dt = (voltage - R current - M angular_velocity) / L
    // d(angular_velocity)/dt = (M current + external_torque) / I
    // d(external_torque)/dt = external_torque_noise;
    
    Matrix<double, 3, 1> x0; x0 <<
      current,
      angular_velocity,
      external_torque;
    
    Matrix<double, 3, 3> A; A <<
      -R/L, -M/L,    0,
       M/I,    0,  1/I,
         0,    0, -0.1;
    // -0.1 is to make A invertible (there's probably some more ideal way to handle this)
    
    Matrix<double, 2, 1> u; u <<
      voltage,
      noise(0);
    Matrix<double, 3, 2> B; B <<
      1/L, 0,
        0, 0,
        0, 1;
    
    Matrix<double, 3, 1> x_new = (A * dt).exp() * x0 +
      A.inverse() * ((A * dt).exp() - Matrix<double, 3, 3>::Identity()) * B * u;
    
    
    
    //std::cout << "external_torque: " << external_torque << std::endl;
    //std::cout << "A*x0: " << A*x0 << std::endl;
    //std::cout << "x_new: " << x_new << std::endl;
    
    return State(
      t,
      x_new(0),
      x_new(1),
      x_new(2),
      R, L, M, I);
  }
  
  DeltaType operator-(const State &other) const {
    return (DeltaType() <<
      current - other.current,
      angular_velocity - other.angular_velocity,
      external_torque - other.external_torque,
      R - other.R,
      L - other.L,
      M - other.M,
      I - other.I).finished();
  }
  State operator+(const DeltaType &other) const {
    return State(
      t,
      current + other(CURRENT_OFFSET),
      angular_velocity + other(ANGULAR_VELOCITY_OFFSET),
      external_torque + other(EXTERNAL_TORQUE_OFFSET),
      R + other(R_OFFSET),
      L + other(L_OFFSET),
      M + other(M_OFFSET),
      I + other(I_OFFSET));
  }
};

class StateWithProcessNoise : public State {
private:
  Matrix<double, PREDICT_EXTRA_NOISE_LENGTH, 1> process_noise;
public:
  static const unsigned int SIZE = State::DELTA_SIZE + PREDICT_EXTRA_NOISE_LENGTH;
  StateWithProcessNoise(const State &state,
      Matrix<double, PREDICT_EXTRA_NOISE_LENGTH, 1> noise) :
    State(state), process_noise(process_noise) { }
  Matrix<double, SIZE, 1> operator-(const StateWithProcessNoise &other) const {
    return (Matrix<double, SIZE, 1>() <<
      static_cast<const State&>(*this) - static_cast<const State&>(other),
      process_noise - other.process_noise).finished();
  }
  StateWithProcessNoise operator+(const Matrix<double, SIZE, 1> &other) const {
    return StateWithProcessNoise(
      static_cast<const State&>(*this) + other.segment<State::DELTA_SIZE>(0),
      process_noise + other.segment<PREDICT_EXTRA_NOISE_LENGTH>(State::DELTA_SIZE));
  }
  State predict(ros::Time t, double voltage) const {
    return static_cast<const State&>(*this).predict(t,
      voltage, process_noise);
  }
};

template<int NN>
class StateWithMeasurementNoise : public State {
public:
  Matrix<double, NN, 1> measurement_noise;
  static const unsigned int DELTA_SIZE = NN != Dynamic ? State::DELTA_SIZE + NN : Dynamic;
  typedef Matrix<double, DELTA_SIZE, 1> DeltaType;
  StateWithMeasurementNoise(const State &state, Matrix<double, NN, 1> measurement_noise) :
    State(state), measurement_noise(measurement_noise) { }
  DeltaType operator-(const StateWithMeasurementNoise<NN> &other) {
    return (DeltaType() <<
        static_cast<const State&>(*this) - static_cast<const State&>(other),
        measurement_noise - other.measurement_noise).finished();
  }
  StateWithMeasurementNoise<NN> operator+(DeltaType &other) const {
    return StateWithMeasurementNoise<NN>(
      static_cast<const State&>(*this) + other.template segment<State::DELTA_SIZE>(0),
      measurement_noise + other.tail(other.rows() - State::DELTA_SIZE));
  }
};

struct AugmentedState : public State {
  typedef Matrix<double, State::DELTA_SIZE, State::DELTA_SIZE> CovType;
  CovType cov;
  AugmentedState(const State &state, const CovType &cov) :
    State(state), cov(cov/2 + cov.transpose()/2) {
    assert_none_nan(cov);
  }
  
  AugmentedState predict(ros::Time t, double voltage) const {
    const unsigned int NOISE_LENGTH = PREDICT_EXTRA_NOISE_LENGTH;
    
    StateWithProcessNoise mean = StateWithProcessNoise(
      static_cast<const State&>(*this),
      Matrix<double, PREDICT_EXTRA_NOISE_LENGTH, 1>::Zero());
    
    typedef Matrix<double, State::DELTA_SIZE + NOISE_LENGTH,
      State::DELTA_SIZE + NOISE_LENGTH> AugmentedMatrixType;
    AugmentedMatrixType Pa = AugmentedMatrixType::Zero();
    Pa.block<State::DELTA_SIZE, State::DELTA_SIZE>(0, 0) = cov;
    Pa.block<PREDICT_EXTRA_NOISE_LENGTH, PREDICT_EXTRA_NOISE_LENGTH>(
      State::DELTA_SIZE, State::DELTA_SIZE) = get_extra_noise_cov();
    
    UnscentedTransform<State, State::DELTA_SIZE,
      StateWithProcessNoise, State::DELTA_SIZE + NOISE_LENGTH> res(
        boost::bind(&StateWithProcessNoise::predict, _1, t, voltage),
        mean, Pa);
    
    return AugmentedState(res.mean, res.cov);
  }
  
  template <int N, int NN>
  AugmentedState update(
      const boost::function<Matrix<double, N, 1> (StateWithMeasurementNoise<NN>)> &observe,
      const Matrix<double, NN, NN> &noise_cov) const {
    unsigned int realNN = NN != Dynamic ? NN : noise_cov.rows();
    assert(noise_cov.rows() == noise_cov.cols());
    
    StateWithMeasurementNoise<NN> mean = StateWithMeasurementNoise<NN>(
      static_cast<const State&>(*this), Matrix<double, NN, 1>::Zero(realNN));
    typedef Matrix<double, NN != Dynamic ? State::DELTA_SIZE + NN : Dynamic,
                           NN != Dynamic ? State::DELTA_SIZE + NN : Dynamic>
      AugmentedMatrixType;
    AugmentedMatrixType Pa = AugmentedMatrixType::Zero(
      State::DELTA_SIZE + realNN, State::DELTA_SIZE + realNN);
    Pa.template block<State::DELTA_SIZE, State::DELTA_SIZE>(0, 0) = cov;
    Pa.template bottomRightCorner(realNN, realNN) = noise_cov;
    
    UnscentedTransform<Matrix<double, N, 1>, N,
      StateWithMeasurementNoise<NN>, NN != Dynamic ? State::DELTA_SIZE + NN : Dynamic>
      res(observe, mean, Pa);
    unsigned int realN = res.mean.rows();
    
    Matrix<double, State::DELTA_SIZE, N> K =
      res.cross_cov.transpose().template topLeftCorner(State::DELTA_SIZE, realN) *
      res.cov.inverse();
    
    State new_state = static_cast<const State&>(*this) + K*-res.mean;
    CovType new_cov = cov - K*res.cov*K.transpose();
    
    return AugmentedState(new_state, new_cov);
  }
};

Matrix<double, 1, 1> current_cov = (Matrix<double, 1, 1>() << pow(0.1, 2)).finished();
Matrix<double, 1, 1> current_observer(double measured_current,
    const StateWithMeasurementNoise<1> &state) {
  return scalar_matrix(state.current + state.measurement_noise(0) - measured_current);
}

Matrix<double, 1, 1> angular_velocity_cov = (Matrix<double, 1, 1>() << pow(0.1, 2)).finished();
Matrix<double, 1, 1> angular_velocity_observer(double measured_angular_velocity,
    const StateWithMeasurementNoise<1> &state) {
  return scalar_matrix(state.angular_velocity + state.measurement_noise(0) - measured_angular_velocity);
}

State state_after_current_optimally_steered_to_desired_current(State const &s, double desired_current) {
  double steer_voltage = s.current < desired_current ? 11 : -11;
  double time_to_desired_current = 0;
  for(int i = 0; i < 10; i++) {
    State tmp = static_cast<const State&>(s);
    tmp = tmp.predict(tmp.t + ros::Duration(time_to_desired_current), steer_voltage, scalar_matrix(0));
    time_to_desired_current -= (tmp.current - desired_current) / tmp.dcurrent_over_dt(steer_voltage);
    std::cout << "test " << i << " " << tmp.current << " " << desired_current << " " << time_to_desired_current << std::endl;
  }
  
  return s.predict(s.t + ros::Duration(time_to_desired_current), steer_voltage, scalar_matrix(0));
}

int main() {
  double R = 8.6; // ohm
  double tau = 0.3e-3; // s
  double M = 0.583568125; // webers = 11 volt / (3 revolutions/second converted to rad/s)
  double I = 2.26796185e-5; // kg m2 = (half pound) * ((1 cm)^2)
  double external_torque = 0.4;
  State s(ros::Time(1), 0, 0, external_torque, R, R*tau, M, I);
  
  State::DeltaType stdev = (State::DeltaType() <<
    1,1,2, 5, .5e-3, .5, .5e-5).finished();
  AugmentedState::CovType tmp = stdev.asDiagonal();
  AugmentedState k(
    State(ros::Time(1), 0, 0, 0, 10, 1e-3, 1, 1e-5),
    tmp*tmp);
  
  double pwm_freq = 100e3;
  double desired_angular_velocity = 5;
  bool going_up = true;
  while(true) {
    double desired_angular_acceleration;
    if(going_up) {
      desired_angular_velocity += 0.01;
      desired_angular_acceleration = 0.01 * pwm_freq;
    } else {
      desired_angular_velocity -= 0.01;
      desired_angular_acceleration = -0.01 * pwm_freq;
    }
    if(desired_angular_velocity < 0) {
      desired_angular_velocity = 0;
      going_up = true;
    }
    if(desired_angular_velocity > 10) {
      desired_angular_velocity = 5;
      going_up = false;
    }
    
    double desired_current =
      (desired_angular_acceleration*k.I - k.external_torque)/k.M;
    
    std::vector<double> voltage_choices{-11., 0., 11.};
    double best_voltage;
    boost::optional<double> best_error = boost::none;
    BOOST_FOREACH(double voltage_choice, voltage_choices) {
      State after_voltage = static_cast<State const &>(k).predict(k.t + ros::Duration(1/pwm_freq), voltage_choice, scalar_matrix(0));
      State new_state = state_after_current_optimally_steered_to_desired_current(after_voltage, desired_current);
      double extrapolated_desired_angular_velocity = desired_angular_velocity + (new_state.t - k.t).toSec() * desired_angular_acceleration;
      double error = fabs(new_state.angular_velocity - extrapolated_desired_angular_velocity);
      std::cout << "CHOICE: " << voltage_choice << " " << error << std::endl;
      if(!best_error || error < *best_error) {
        best_voltage = voltage_choice;
        best_error = error;
      }
    }
    double voltage = best_voltage;
    
    s = s.predict(s.t + ros::Duration(1/pwm_freq), voltage, scalar_matrix(external_torque/10));
    k = k.predict(k.t + ros::Duration(1/pwm_freq), voltage);
    k = k.update<1, 1>(boost::bind(current_observer, s.current, _1), current_cov);
    
    /*for(int i = 0; i < 10; i++) {
      s = s.predict(s.t + ros::Duration((1-duty)/pwm_freq/10), 0, scalar_matrix(external_torque/10));
      k = k.predict(k.t + ros::Duration((1-duty)/pwm_freq/10), 0);
      k = k.update<1, 1>(boost::bind(current_observer, s.current, _1), current_cov);
    }*/
    
    k = k.update<1, 1>(boost::bind(angular_velocity_observer, s.angular_velocity, _1), angular_velocity_cov);
    
    std::cout << "desired_angular_velocity: " << desired_angular_velocity << std::endl;
    std::cout << "desired_current: " << desired_current << std::endl;
    std::cout << "voltage: " << voltage << std::endl;
    std::cout << "current: " << s.current << " " << k.current << std::endl;
    std::cout << "angular_velocity: " << s.angular_velocity << " " << k.angular_velocity << std::endl;
    std::cout << "external_torque: " << s.external_torque << " " << k.external_torque << std::endl;
    std::cout << "R: " << s.R << " " << k.R << std::endl;
    std::cout << "L: " << s.L << " " << k.L << std::endl;
    std::cout << "M: " << s.M << " " << k.M << std::endl;
    std::cout << "I: " << s.I << " " << k.I << std::endl;
    std::cout << "PLOT: " << desired_angular_velocity << " " << k.angular_velocity << " " << s.angular_velocity << " " << voltage << std::endl;
    std::cout << std::endl;
  }

  return 0;
}
