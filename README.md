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

* Required logging topics
  * actuator\_controls > 200 Hz
  * sensor\_combined > 200 Hz

### Commandline

```bash
ulog_tools_pid_design --url http://review.px4.io/download?log=d3980cdd-c8ef-478b-a838-fba9956009c4 out.gains
```

#### Result

```json
{
  "MC_PITCHRATE_D": 0.0034238889791543717,
  "MC_PITCHRATE_I": 0.07823542271951296,
  "MC_PITCHRATE_P": 0.08827774216535533,
  "MC_PITCH_P": 7.0339592673305855,
  "MC_ROLLRATE_D": 0.0033358807670637275,
  "MC_ROLLRATE_I": 0.10385472999114756,
  "MC_ROLLRATE_P": 0.10710041094076796,
  "MC_ROLL_P": 5.304756498388342,
  "MC_YAWRATE_D": 0.0036348770832720268,
  "MC_YAWRATE_I": 0.9300066286051631,
  "MC_YAWRATE_P": 1.0085375280948274,
  "MC_YAW_P": 6.985848146126574,
  "pitch": {
    "model": {
      "delay": 0.04453746192794547,
      "f_s": 246.98309072475325,
      "fit": 0.9795666272678758,
      "gain": 142.44997857292887,
      "sample_delay": 11
    },
    "t_end": 25,
    "t_start": 20
  },
  "roll": {
    "model": {
      "delay": 0.0404886017526777,
      "f_s": 246.98309072475325,
      "fit": 0.9944586094307987,
      "gain": 171.34199377037035,
      "sample_delay": 10
    },
    "t_end": 15,
    "t_start": 10
  },
  "yaw": {
    "model": {
      "delay": 0.0,
      "f_s": 246.98309072475325,
      "fit": 0.7879995939132498,
      "gain": 13.572575231076963,
      "sample_delay": 0
    },
    "t_end": 60,
    "t_start": 55
  }
}
```

## Algorithms

* System Identification: Slides a window across the entire log, default 5 seconds, returns the model with the best fit,  fit = (1- error variance/ signal variance)
* Control Design:  LQR with output feedback approach for structured control system optimization

