from uf_smach import missions

missions.register_config('test', dict(
        factory='test',
        timeout=120,
        iterations=4))

missions.register_config('short_test', dict(
        factory='test',
        timeout=20,
        iterations=2))
