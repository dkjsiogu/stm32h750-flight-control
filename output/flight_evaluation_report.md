# Flight Control Closed-Loop Evaluation

- policy: generated static MLP only
- loop: SpeedController -> ModelAdapter -> TorqueController -> PWM -> motor lag -> host truth dynamics
- sensors: delayed estimated state with gyro bias, drift, jitter, position/velocity bias
- stable scenarios: 5/5
- average score: 84.5/100

| Scenario | Stable | Score | Velocity RMS | Alt Drift | Max Tilt | PWM Sat | Recovery | Final z |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| hover_wind_bias | yes | 92.428 | 0.027 | 0.017 | 0.022 | 0.000 | 0.000 | 1.183 |
| forward_cruise | yes | 88.055 | 0.021 | 0.018 | 0.033 | 0.000 | 0.704 | 1.184 |
| gust_recovery | yes | 78.316 | 0.171 | 0.018 | 0.076 | 0.000 | 0.000 | 1.183 |
| payload_motor_lag | yes | 78.252 | 0.058 | 0.354 | 0.012 | 0.000 | 0.000 | 1.083 |
| climb_turn_cruise | yes | 85.562 | 0.081 | 0.252 | 0.024 | 0.000 | 0.152 | 2.188 |
