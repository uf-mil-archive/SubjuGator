#include <boost/foreach.hpp>

#include <ros/ros.h>
#include <unsupported/Eigen/MatrixFunctions>

#include <odom_estimator/unscented_transform.h>
#include <odom_estimator/kalman.h>

using namespace Eigen;
using namespace odom_estimator;


ODOM_ESTIMATOR_DEFINE_MANIFOLD_BEGIN(State,
  (ros::Time, t)
  ,
  (WrappedScalar, current)
  (WrappedScalar, angular_velocity)
  (WrappedScalar, external_torque)
  
  (WrappedScalar, R)
  (WrappedScalar, L)
  (WrappedScalar, M)
  (WrappedScalar, I)
)
  double dcurrent_over_dt(double voltage) const {
    return (voltage - R * current - M * angular_velocity) / L;
  }
  
  State predict(ros::Time t, double voltage, double external_torque_derivative=0) const {
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
      external_torque_derivative;
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
ODOM_ESTIMATOR_DEFINE_MANIFOLD_END()

class StateUpdater : public UnscentedTransformDistributionFunction<State, State, Vec<1> > {
  ros::Time t;
  double voltage;
  
  GaussianDistribution<Vec<1> > get_extra_distribution() const {
    return GaussianDistribution<Vec<1> >(
      Vec<1>::Zero(),
      scalar_matrix(100));
  }

public:
  State apply(State const &state, Vec<1> const &noise) const {
    return state.predict(t, voltage, noise(0));
  }
  StateUpdater(ros::Time t, double voltage) :
    t(t), voltage(voltage) {
  }
};

class CurrentErrorObserver : public UnscentedTransformDistributionFunction<State, WrappedScalar, Vec<1> > {
  double measured_current;
  GaussianDistribution<Vec<1> > get_extra_distribution() const {
    return GaussianDistribution<Vec<1> >(
      Vec<1>::Zero(),
      (Matrix<double, 1, 1>() << pow(0.1, 2)).finished());
  }
  WrappedScalar apply(State const &state, Vec<1> const &noise) const {
    return state.current + noise(0) - measured_current;
  }
public:
  CurrentErrorObserver(double measured_current) :
    measured_current(measured_current) {
  }
};

class AngularVelocityErrorObserver : public UnscentedTransformDistributionFunction<State, WrappedScalar, Vec<1> > {
  double measured_angular_velocity;
  GaussianDistribution<Vec<1> > get_extra_distribution() const {
    return GaussianDistribution<Vec<1> >(
      Vec<1>::Zero(),
      (Matrix<double, 1, 1>() << pow(0.1, 2)).finished());
  }
  WrappedScalar apply(State const &state, Vec<1> const &noise) const {
    return state.angular_velocity + noise(0) - measured_angular_velocity;
  }
public:
  AngularVelocityErrorObserver(double measured_angular_velocity) :
    measured_angular_velocity(measured_angular_velocity) {
  }
};

State state_after_current_optimally_steered_to_desired_current(State const &s, double desired_current) {
  double steer_voltage = s.current < desired_current ? 11 : -11;
  double time_to_desired_current = 0;
  for(int i = 0; i < 10; i++) {
    State tmp = static_cast<const State&>(s);
    tmp = tmp.predict(tmp.t + ros::Duration(time_to_desired_current), steer_voltage);
    time_to_desired_current -= (static_cast<double>(tmp.current) - desired_current) / tmp.dcurrent_over_dt(steer_voltage);
    std::cout << "test " << i << " " << tmp.current << " " << desired_current << " " << time_to_desired_current << std::endl;
  }
  
  return s.predict(s.t + ros::Duration(time_to_desired_current), steer_voltage);
}

int main() {
  double R = 8.6; // ohm
  double tau = 0.3e-3; // s
  double M = 0.583568125; // webers = 11 volt / (3 revolutions/second converted to rad/s)
  double I = 2.26796185e-5; // kg m2 = (half pound) * ((1 cm)^2)
  double external_torque = 0.4;
  State s(ros::Time(1), 0, 0, external_torque, R, R*tau, M, I);
  
  Vec<State::RowsAtCompileTime> stdev = (Vec<State::RowsAtCompileTime>() <<
    1,1,2, 5, .5e-3, .5, .5e-5).finished();
  SqMat<State::RowsAtCompileTime> tmp = stdev.asDiagonal();
  GaussianDistribution<State> k(
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
      (desired_angular_acceleration*k.mean.I - k.mean.external_torque)/k.mean.M;
    
    std::vector<double> voltage_choices{-11., 0., 11.};
    double best_voltage;
    boost::optional<double> best_error = boost::none;
    BOOST_FOREACH(double voltage_choice, voltage_choices) {
      State after_voltage = k.mean.predict(k.mean.t + ros::Duration(1/pwm_freq), voltage_choice);
      State new_state = state_after_current_optimally_steered_to_desired_current(after_voltage, desired_current);
      double extrapolated_desired_angular_velocity = desired_angular_velocity + (new_state.t - k.mean.t).toSec() * desired_angular_acceleration;
      double error = fabs(static_cast<double>(new_state.angular_velocity) - extrapolated_desired_angular_velocity);
      std::cout << "CHOICE: " << voltage_choice << " " << error << std::endl;
      if(!best_error || error < *best_error) {
        best_voltage = voltage_choice;
        best_error = error;
      }
    }
    double voltage = best_voltage;
    
    s = s.predict(s.t + ros::Duration(1/pwm_freq), voltage, external_torque/10);
    k = StateUpdater(k.mean.t + ros::Duration(1/pwm_freq), voltage)(k);
    k = kalman_update(CurrentErrorObserver(s.current), k);
    
    /*for(int i = 0; i < 10; i++) {
      s = s.predict(s.t + ros::Duration((1-duty)/pwm_freq/10), 0, external_torque/10);
      k = k.predict(k.t + ros::Duration((1-duty)/pwm_freq/10), 0);
      k = k.update<1, 1>(boost::bind(current_observer, s.current, _1), current_cov);
    }*/
    
    k = kalman_update(AngularVelocityErrorObserver(s.angular_velocity), k);
    
    std::cout << "desired_angular_velocity: " << desired_angular_velocity << std::endl;
    std::cout << "desired_current: " << desired_current << std::endl;
    std::cout << "voltage: " << voltage << std::endl;
    std::cout << "current: " << s.current << " " << k.mean.current << std::endl;
    std::cout << "angular_velocity: " << s.angular_velocity << " " << k.mean.angular_velocity << std::endl;
    std::cout << "external_torque: " << s.external_torque << " " << k.mean.external_torque << std::endl;
    std::cout << "R: " << s.R << " " << k.mean.R << std::endl;
    std::cout << "L: " << s.L << " " << k.mean.L << std::endl;
    std::cout << "M: " << s.M << " " << k.mean.M << std::endl;
    std::cout << "I: " << s.I << " " << k.mean.I << std::endl;
    std::cout << "PLOT: " << desired_angular_velocity << " " << k.mean.angular_velocity << " " << s.angular_velocity << " " << voltage << std::endl;
    std::cout << std::endl;
  }

  return 0;
}
