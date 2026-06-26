# Flight Control Closed-Loop Evaluation

- policy: generated static MLP only
- loop: SpeedController -> ModelAdapter -> TorqueController -> PWM -> motor lag -> host truth dynamics
- sensors: delayed estimated state with gyro bias, drift, jitter, position/velocity bias
- stable scenarios: 5/5
- average score: 75.6/100

| Scenario | Stable | Score | Velocity RMS | Alt Drift | Max Tilt | PWM Sat | Recovery | Final z |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| hover_wind_bias | yes | 87.321 | 0.043 | 0.021 | 0.058 | 0.000 | 0.000 | 1.184 |
| forward_cruise | yes | 82.457 | 0.045 | 0.021 | 0.064 | 0.000 | 0.756 | 1.182 |
| gust_recovery | yes | 64.634 | 0.189 | 0.021 | 0.107 | 0.000 | 1.448 | 1.180 |
| payload_motor_lag | yes | 64.043 | 0.125 | 0.355 | 0.140 | 0.000 | 0.000 | 1.083 |
| climb_turn_cruise | yes | 79.382 | 0.102 | 0.275 | 0.102 | 0.000 | 0.180 | 2.157 |
