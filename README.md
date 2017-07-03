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
  "MC_PITCHRATE_D": 0.03578103446587077,
  "MC_PITCHRATE_I": 0.08625567596884964,
  "MC_PITCHRATE_P": 0.14274594001386864,
  "MC_PITCH_P": 2.7059694647877106,
  "MC_ROLLRATE_D": 0.009080890302959081,
  "MC_ROLLRATE_I": 0.1991381399373928,
  "MC_ROLLRATE_P": 0.2010551782440861,
  "MC_ROLL_P": 4.5975117713393,
  "MC_YAWRATE_D": 0.017251069591687748,
  "MC_YAWRATE_I": 0.19498248018478978,
  "MC_YAWRATE_P": 0.1892431933790533,
  "MC_YAW_P": 3.598452484267229,
  "pitch": {
    "model": {
      "delay": 0.11159139554746923,
      "f_s": 232.9928743380578,
      "fit": 0.4776096073601829,
      "gain": 22.529516174300294,
      "sample_delay": 26
    },
    "t_end": 10,
    "t_start": 5
  },
  "roll": {
    "model": {
      "delay": 0.06867162802921184,
      "f_s": 232.9928743380578,
      "fit": 0.7487414753309868,
      "gain": 63.19214713528321,
      "sample_delay": 16
    },
    "t_end": 10,
    "t_start": 5
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

