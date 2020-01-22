# ulog_tools
ulog analysis tools

## Requirements

* Python 3
* see environment.yml for dependencies

## Install

```bash
pip3 install ulog_tools
```

Or using conda

```bash
conda env create -f environment.yml
conda activate ulog_tools
```

## Usage

### Log File

* Required logging topics
  * actuator\_controls > 200 Hz
  * sensor\_combined > 200 Hz

### Commandline

```bash
ulog_tools_pid_design --url https://logs.px4.io/download?log=0467b169-aec0-44d0-bbd0-a42cce863acf  --verbose --plot out.txt
```

#### Result

```json
{
  "MC_PITCHRATE_D": 0.007986111763821054,
  "MC_PITCHRATE_I": 1.4199280160960568,
  "MC_PITCHRATE_P": 0.3010445756612024,
  "MC_PITCH_P": 5.474760010163753,
  "MC_ROLLRATE_D": 0.008747392635960064,
  "MC_ROLLRATE_I": 1.1962720090900754,
  "MC_ROLLRATE_P": 0.29499919522081797,
  "MC_ROLL_P": 5.361772007628582,
  "MC_YAWRATE_D": 0.011026584612036192,
  "MC_YAWRATE_I": 1.9736695282813295,
  "MC_YAWRATE_P": 0.4805348843295458,
  "MC_YAW_P": 5.317066206669442,
  "pitch": {
    "model": {
      "delay": 0.05778450098871699,
      "f_s": 224.9738213113302,
      "fit": 0.6820218656710324,
      "gain": 108.63532791159108,
      "sample_delay": 13
    },
    "t_end": 110,
    "t_start": 105
  },
  "roll": {
    "model": {
      "delay": 0.06667442421775038,
      "f_s": 224.9738213113302,
      "fit": 0.7482813628055249,
      "gain": 100.80854701682381,
      "sample_delay": 15
    },
    "t_end": 110,
    "t_start": 105
  },
  "yaw": {
    "model": {
      "delay": 0.04889457775968361,
      "f_s": 224.9738213113302,
      "fit": 0.43330939920399325,
      "gain": 72.43734245165282,
      "sample_delay": 11
    },
    "t_end": 110,
    "t_start": 105
  }
}
```

## Algorithms

* System Identification: Slides a window across the entire log, default 5 seconds, returns the model with the best fit,  fit = (1- error variance/ signal variance)
* Control Design:  LQR with output feedback approach for structured control system optimization

