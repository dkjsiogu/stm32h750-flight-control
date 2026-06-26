# Flight Control Closed-Loop Evaluation

- policy: generated static MLP only
- loop: SpeedController -> ModelAdapter -> TorqueController -> PWM -> motor lag -> host truth dynamics
- sensors: delayed estimated state with gyro bias, drift, jitter, position/velocity bias
- stable scenarios: 5/5
- average score: 84.2/100

| Scenario | Stable | Score | Velocity RMS | Alt Drift | Max Tilt | PWM Sat | Recovery | Final z |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| hover_wind_bias | yes | 92.414 | 0.027 | 0.017 | 0.022 | 0.000 | 0.000 | 1.183 |
| forward_cruise | yes | 87.939 | 0.023 | 0.017 | 0.029 | 0.000 | 0.720 | 1.184 |
| gust_recovery | yes | 77.511 | 0.179 | 0.017 | 0.077 | 0.000 | 0.000 | 1.184 |
| payload_motor_lag | yes | 77.771 | 0.059 | 0.355 | 0.019 | 0.000 | 0.000 | 1.083 |
| climb_turn_cruise | yes | 85.467 | 0.082 | 0.251 | 0.023 | 0.000 | 0.156 | 2.189 |
