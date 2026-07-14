import io
import sys
import markdown
import numpy as np
import matplotlib.pyplot as plt
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.tests.test_following_distance import desired_follow_distance
from openpilot.tools.longitudinal_maneuvers.maneuver_helpers import Axis, axis_labels
from openpilot.selfdrive.test.longitudinal_maneuvers.maneuver import Maneuver


def get_html_from_results(results, labels, AXIS):
  fig, ax = plt.subplots(figsize=(16, 8))
  for idx, key in enumerate(results.keys()):
    ax.plot(results[key][:, Axis.TIME], results[key][:, AXIS], label=labels[idx])

  ax.set_xlabel(axis_labels[Axis.TIME])
  ax.set_ylabel(axis_labels[AXIS])
  ax.legend(bbox_to_anchor=(1.02, 1), loc='upper left', borderaxespad=0)
  ax.grid(True, linestyle='--', alpha=0.7)
  ax.text(-0.075, 0.5, '.', transform=ax.transAxes, color='none')

  fig_buffer = io.StringIO()
  fig.savefig(fig_buffer, format='svg', bbox_inches='tight')
  plt.close(fig)
  return fig_buffer.getvalue() + '<br/>'

def run_maneuver_set(name, maneuvers, axes):
  """Run a set of maneuvers in both cruise and e2e (long) mode and return the plots."""
  htmls = [markdown.markdown('# ' + name)]
  for e2e in (False, True):
    results = {}
    labels = []
    for label, man in maneuvers:
      man.e2e = e2e
      valid, results[label] = man.evaluate()
      labels.append(label)
    mode = 'e2e (long)' if e2e else 'cruise'
    htmls.append(markdown.markdown(f'## {mode} mode'))
    for axis in axes:
      htmls.append(get_html_from_results(results, labels, axis))
  return htmls

def generate_mpc_tuning_report():
  htmls = []

  maneuvers = []
  for lead_accel in np.linspace(1.0, 4.0, 4):
    maneuvers.append((
      f'{lead_accel} m/s^2 lead acceleration',
      Maneuver(
        '',
        duration=11,
        initial_speed=0.0,
        lead_relevancy=True,
        initial_distance_lead=desired_follow_distance(0.0, 0.0),
        speed_lead_values=[0.0, 10 * lead_accel],
        cruise_values=[100, 100],
        prob_lead_values=[1.0, 1.0],
        breakpoints=[1., 11],
      ),
    ))
  htmls += run_maneuver_set('Resuming behind lead', maneuvers, [Axis.EGO_V, Axis.EGO_A])


  maneuvers = []
  name = 'Approaching stopped car from 140m'
  for speed in np.arange(0, 45, 5):
    maneuvers.append((
      f'{speed} m/s approach speed',
      Maneuver(
        name,
        duration=30.,
        initial_speed=float(speed),
        lead_relevancy=True,
        initial_distance_lead=140.,
        speed_lead_values=[0.0, 0.],
        breakpoints=[0., 30.],
      ),
    ))
  htmls += run_maneuver_set(name, maneuvers, [Axis.EGO_A, Axis.D_REL])


  maneuvers = []
  speed = np.int64(10)
  for oscil in np.arange(0, 10, 1):
    maneuvers.append((
      f'{oscil} m/s oscillation size',
      Maneuver(
        '',
        duration=30.,
        initial_speed=float(speed),
        lead_relevancy=True,
        initial_distance_lead=desired_follow_distance(speed, speed),
        speed_lead_values=[speed, speed, speed - oscil, speed + oscil, speed - oscil, speed + oscil, speed - oscil],
        breakpoints=[0., 2., 5, 8, 15, 18, 25.],
      ),
    ))
  htmls += run_maneuver_set('Following 5s (triangular) oscillating lead', maneuvers,
                            [Axis.D_REL, Axis.EGO_V, Axis.EGO_A])


  maneuvers = []
  speed = np.int64(10)
  duration = float(30)
  f_osc = 1. / 5
  for oscil in np.arange(0, 10, 1):
    bps = DT_MDL * np.arange(int(duration / DT_MDL))
    lead_speeds = speed + oscil * np.sin(2 * np.pi * f_osc * bps)
    maneuvers.append((
      f'{oscil} m/s oscillation size',
      Maneuver(
        '',
        duration=duration,
        initial_speed=float(speed),
        lead_relevancy=True,
        initial_distance_lead=desired_follow_distance(speed, speed),
        speed_lead_values=lead_speeds,
        breakpoints=bps,
      ),
    ))
  htmls += run_maneuver_set('Following 5s (sinusoidal) oscillating lead', maneuvers,
                            [Axis.D_REL, Axis.EGO_V, Axis.EGO_A])


  maneuvers = []
  for distance in np.arange(20, 140, 10):
    maneuvers.append((
      f'{distance} m initial distance',
      Maneuver(
        '',
        duration=50,
        initial_speed=30.0,
        lead_relevancy=True,
        initial_distance_lead=distance,
        speed_lead_values=[30.0],
        breakpoints=[0.],
      ),
    ))
  htmls += run_maneuver_set('Speed profile when converging to steady state lead at 30m/s', maneuvers,
                            [Axis.EGO_V, Axis.D_REL])


  maneuvers = []
  for distance in np.arange(20, 140, 10):
    maneuvers.append((
      f'{distance} m initial distance',
      Maneuver(
        '',
        duration=50,
        initial_speed=20.0,
        lead_relevancy=True,
        initial_distance_lead=distance,
        speed_lead_values=[20.0],
        breakpoints=[0.],
      ),
    ))
  htmls += run_maneuver_set('Speed profile when converging to steady state lead at 20m/s', maneuvers,
                            [Axis.EGO_V, Axis.D_REL])


  maneuvers = []
  for stop_time in np.arange(4, 14, 1):
    maneuvers.append((
      f'{stop_time} seconds stop time',
      Maneuver(
        '',
        duration=30,
        initial_speed=30.0,
        cruise_values=[30.0, 30.0, 30.0],
        lead_relevancy=True,
        initial_distance_lead=60.0,
        speed_lead_values=[30.0, 30.0, 0.0],
        breakpoints=[0., 5., 5 + stop_time],
      ),
    ))
  htmls += run_maneuver_set('Following car at 30m/s that comes to a stop', maneuvers,
                            [Axis.EGO_A, Axis.D_REL])


  maneuvers = []
  for speed in np.arange(0, 40, 5):
    maneuvers.append((
      f'{speed} m/s speed',
      Maneuver(
        '',
        duration=20,
        initial_speed=float(speed),
        cruise_values=[speed, speed, speed],
        lead_relevancy=True,
        initial_distance_lead=desired_follow_distance(speed, speed) / 2,
        speed_lead_values=[speed, speed, speed],
        prob_lead_values=[0.0, 0.0, 1.0],
        breakpoints=[0., 5.0, 5.01],
      ),
    ))
  htmls += run_maneuver_set('Response to cut-in at half follow distance', maneuvers,
                            [Axis.EGO_A, Axis.D_REL])


  maneuvers = []
  for speed in np.arange(0, 40, 5):
    maneuvers.append((
      f'{speed} m/s speed',
      Maneuver(
        '',
        duration=60,
        initial_speed=0.0,
        lead_relevancy=True,
        initial_distance_lead=desired_follow_distance(0.0, 0.0),
        speed_lead_values=[0.0, 0.0, speed],
        prob_lead_values=[1.0, 1.0, 1.0],
        breakpoints=[0., 1.0, speed / 2],
      ),
    ))
  htmls += run_maneuver_set('Follow a lead that accelerates at 2m/s^2 until steady state speed', maneuvers,
                            [Axis.EGO_V, Axis.EGO_A])


  maneuvers = []
  for speed in np.arange(0, 40, 5):
    maneuvers.append((
      f'{speed} m/s speed',
      Maneuver(
        '',
        duration=50,
        initial_speed=0.0,
        lead_relevancy=True,
        initial_distance_lead=desired_follow_distance(0.0, 0.0),
        speed_lead_values=[0.0, 0.0],
        cruise_values=[0.0, speed],
        prob_lead_values=[0.0, 0.0],
        breakpoints=[1., 1.01],
      ),
    ))
  htmls += run_maneuver_set('From stop to cruise', maneuvers, [Axis.EGO_V, Axis.EGO_A])


  maneuvers = []
  for speed in np.arange(10, 40, 5):
    maneuvers.append((
      f'{speed} m/s speed',
      Maneuver(
        '',
        duration=50,
        initial_speed=float(speed),
        lead_relevancy=True,
        initial_distance_lead=desired_follow_distance(0.0, 0.0),
        speed_lead_values=[0.0, 0.0],
        cruise_values=[speed, 10.0],
        prob_lead_values=[0.0, 0.0],
        breakpoints=[1., 1.01],
      ),
    ))
  htmls += run_maneuver_set('From cruise to min', maneuvers, [Axis.EGO_V, Axis.EGO_A])

  return htmls

if __name__ == '__main__':
  htmls = generate_mpc_tuning_report()

  if len(sys.argv) < 2:
    file_name = 'long_mpc_tune_report.html'
  else:
    file_name = sys.argv[1]

  with open(file_name, 'w') as f:
    f.write(markdown.markdown('# MPC longitudinal tuning report'))
    for html in htmls:
      f.write(html)
