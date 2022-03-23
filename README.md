# RICO MANUAL

## Power on

1. Robot powinien być w docku podłączonym do prądu
2. Odblokować grzyba, wcisnąć lewy przyciski zaczekać chwilę
3. Wcisnąć prawy przycisk, włączy się komputer

## Undocking

1. Laptop z labu powinien mieć automatycznie komunikację z roscore na Rico, puścić message:
`rostopic pub /undocker_server/goal laser_servoing_msgs/UndockActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal: {}"
`

2. Robot powidnien automatycznie wycofać z doku i obrócić się. Jeśli nie, wejdź
przez ssh i sprawdź plik ~/.pal/was_docked.yaml. Jeśli wartość jest False,
robot nie został poprawnie zadokowany. Wyjedź padem i można pracować dalej.

## Docking

1. Podjedź w pobliże bazy dokującej
2. Wyślij wiadomość
`
rostopic pub /go_and_dock/goal dock_charge_sm_msgs/GoAndDockActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  samp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  retry_delay:
    secs: 0
    nsecs: 0
  use_current_pose: true"
`

Parametr use_current_pose należy ustawić na `true`, wtedy od razu
zaczyna szukać doku, zamiast dojeżdżać najpierw w jego pobliże.