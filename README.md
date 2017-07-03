# ulog_tools
ulog analysis tools

## Requirements

* Python 3

## Install

```bash
pip3 install ulog_tools
```

## Usage

### Log File

* Recommended logging topicss
 
 * actuator\_controls : 250 Hz
 * sensor\_combined : 250 Hz

### Commandline

```bash
ulog_tools_pid_design --url http://review.px4.io/download?log=35b27fdb-6a93-427a-b634-72ab45b9609e out.gains
```

#### Result

```json
{
  "MC_PITCHRATE_D": 0.015662896675004697,
  "MC_PITCHRATE_I": 0.4884764564007624,
  "MC_PITCHRATE_P": 0.5110402961942668,
  "MC_PITCH_P": 5.866651469550199,
  "MC_ROLLRATE_D": 0.0063734754545666465,
  "MC_ROLLRATE_I": 0.19844682163683977,
  "MC_ROLLRATE_P": 0.20051094114484688,
  "MC_ROLL_P": 4.584342619034165,
  "MC_YAWRATE_D": 0.017251069591687748,
  "MC_YAWRATE_I": 0.19498248018478978,
  "MC_YAWRATE_P": 0.1892431933790533,
  "MC_YAW_P": 3.598452484267229,
  "pitch": {
    "model": {
      "delay": 0.05150372102190887,
      "f_s": 232.9928743380578,
      "fit": 0.571358281783254,
      "gain": 29.538700377173605,
      "sample_delay": 12
    },
    "t_end": 80,
    "t_start": 75
  },
  "roll": {
    "model": {
      "delay": 0.07296360478103757,
      "f_s": 232.9928743380578,
      "fit": 0.8097024638748593,
      "gain": 45.6867382886041,
      "sample_delay": 17
    },
    "t_end": 105,
    "t_start": 100
  },
  "yaw": {
    "model": {
      "delay": 0.11159139554746923,
      "f_s": 232.9928743380578,
      "fit": 0.8741553922191986,
      "gain": 41.27452742731875,
      "sample_delay": 26
    },
    "t_end": 5,
    "t_start": 0
  }
}
```

