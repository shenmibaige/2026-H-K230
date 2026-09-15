# balance_car_final

This project integrates four completed tasks into one CCS project:

| OLED selection | Source project | Function |
| --- | --- | --- |
| Q2 | `balance_car_v1.3/Q2_CCS_PROJECT` | Smooth line following |
| Q3 | `balance_car_v2.7` | Ball launch and B-point control |
| Q4 | `balance_car_v4.0` | Line following plus ball centering |
| Q6 | `balance_car_v5.0` | Line following plus auto-captured ball target |

The control algorithms and tuning parameters are copied from the original
projects. They are isolated behind mode modules so only the selected mode runs
from the shared 1 ms timer.

## Controls

| Button | Menu state | Running state |
| --- | --- | --- |
| K1 | Select next task (Q2 -> Q3 -> Q4 -> Q6) | Stop current task and return to menu |
| K2 | Start the selected task | Ignored |

The OLED shows the selected task in the menu and the elapsed seconds after
K2 starts a task. The elapsed seconds reset when the task is stopped.

For Q6, the ball can be placed anywhere. After K2 starts the task, the
controller captures the ball position before moving the car and uses that as
the target.

## Import

1. Open CCS.
2. Import the existing project at:
   `F:\MCU\11_TI_MSP\CCS_project\2026project\balance_car_final`
3. Build and flash `Debug/balance_car_final.out`.

The project uses the MSPM0G3507 target and the SysConfig generated files in
`Debug`. If CCS regenerates SysConfig output, no application source changes
are required.
