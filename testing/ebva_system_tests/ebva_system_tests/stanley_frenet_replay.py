"""Finite stamped clearance/odometry rosbag2 fixture, without real producers."""
import argparse
import json
import math
from pathlib import Path
from .resources import share, fixture_data, fixture_path, profile_path
from .stanley_frenet_software import NAMESPACE, inspect_profile


def make_bag(output, fixture):
    import rosbag2_py
    from rclpy.serialization import serialize_message
    from nav_msgs.msg import Odometry
    from ebva_control_msgs.msg import FrenetLateralErrorStamped, RcInput
    prefix='/'+NAMESPACE;origin=round(fixture['origin_ros_s']*1e9);events=[]
    types={prefix+'/state/odometry':'nav_msgs/msg/Odometry',
           prefix+'/state/lateral_error':'ebva_control_msgs/msg/FrenetLateralErrorStamped',
           prefix+'/rc/input':'ebva_control_msgs/msg/RcInput'}
    def stamped(message,ns,frame):
        message.header.stamp.sec,message.header.stamp.nanosec=divmod(ns,1_000_000_000)
        message.header.frame_id=frame;return message
    for index in range(round(fixture['duration_s']/fixture['period_s'])+1):
        t=round(index*fixture['period_s'],6);ns=origin+round(t*1e9)
        phase=next((p for p in fixture['phases'] if p['start']<=t<p['end']),None)
        if phase is None or phase['mode']=='silence':continue
        mode=phase['mode']
        odom_offset,error_offset=((30_000_000,10_000_000) if index%2 else (10_000_000,30_000_000))
        if mode!='odometry_silence':
            stamp=ns-800_000_000 if mode=='stale_odometry' else ns-odom_offset
            state=stamped(Odometry(),stamp,fixture['reference_frame'])
            state.child_frame_id=fixture['body_frame'];state.pose.pose.orientation.w=1.
            state.twist.twist.linear.x=-.6
            events.append((ns,0,prefix+'/state/odometry',state))
        if mode!='lateral_error_silence':
            stamp=ns-120_000_000 if mode=='skew' else ns-error_offset
            error=stamped(FrenetLateralErrorStamped(),stamp,fixture['reference_frame'])
            error.reference_id='unknown_reference' if mode=='wrong_reference' else fixture['reference_id']
            error.body_frame_id=fixture['body_frame'];error.model_point=fixture['model_point']
            error.contract_id=fixture['contract_id']
            error.lateral_error_m=float('nan') if mode=='invalid_error' else phase['lateral_error_m']
            events.append((ns,1,prefix+'/state/lateral_error',error))
    for index in range(round(fixture['duration_s']/fixture['rc_period_s'])+1):
        t=round(index*fixture['rc_period_s'],6);ns=origin+round(t*1e9)
        rc=stamped(RcInput(),ns,'fixture_rc');rc.sequence=index+1
        rc.valid=True;rc.rf_state=RcInput.RF_UNKNOWN;rc.reason='synthetic_replay';rc.throttle_us=1500.
        lo,hi=fixture['manual_rc_interval_s'];rc.steering_us=1700. if lo<=t<hi else 1500.
        events.append((ns,2,prefix+'/rc/input',rc))
    writer=rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(output/'input_bag'),storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr','cdr'))
    for topic,name in types.items():
        writer.create_topic(rosbag2_py.TopicMetadata(name=topic,type=name,serialization_format='cdr'))
    for ns,_,topic,message in sorted(events,key=lambda event:(event[0],event[1])):
        writer.write(topic,serialize_message(message),ns)
    del writer
    return dict(kind=fixture['kind'],messages=len(events),topics=types)


def subscriptions(record):
    from nav_msgs.msg import Odometry
    from ebva_control_msgs.msg import FrenetLateralErrorStamped
    from ackermann_msgs.msg import AckermannDriveStamped
    from diagnostic_msgs.msg import DiagnosticArray
    def status(message):
        if not message.status:return
        record('path_status',message,{'reason':message.status[0].message,
            'values':{kv.key:kv.value for kv in message.status[0].values}})
    return [(Odometry,'state/odometry',lambda m:record('source_odometry',m,
                {'body':m.child_frame_id,'vx':m.twist.twist.linear.x,'vy':m.twist.twist.linear.y})),
            (FrenetLateralErrorStamped,'state/lateral_error',lambda m:record('source_error',m,
                {'reference_id':m.reference_id,'body':m.body_frame_id,'point':m.model_point,
                 'contract_id':m.contract_id,'error_m':m.lateral_error_m})),
            (AckermannDriveStamped,'control/autonomy/ackermann',lambda m:record('ackermann',m,
                {'speed':m.drive.speed,'steering':m.drive.steering_angle})),
            (DiagnosticArray,'control/path_status',status),
            (DiagnosticArray,'control/ackermann_status',lambda m:record('ackermann_status',m,{}))]


def verify(events, fixture):
    from ebva_control_msgs.msg import SelectedControl
    body,parent='fixture_rear_axle_flu','fixture_map'
    def rows(kind,lo,hi):
        found=[event for event in events if event['kind']==kind and lo<=event['t_s']<hi]
        if not found:raise RuntimeError(f'No {kind} in {lo}..{hi}')
        return found
    commands=rows('ackermann',.8,4.4)
    if not all(abs(event['speed']+.6)<3e-8 and abs(event['steering'])<=.4 for event in commands):
        raise RuntimeError('Original signed speed/steering limits changed')
    for lo,hi,sign in ((1.7,2.2,-1),(2.7,3.2,1)):
        if not all(sign*event['steering']>.01 for event in rows('ackermann',lo,hi)):
            raise RuntimeError('Original clearance inversion/law sign not exercised')
    if not all(event['mode']==SelectedControl.MANUAL for event in rows('selection',3.75,3.95)):
        raise RuntimeError('RC did not take integral priority')
    if not all(event['erpm']==0 for event in rows('motor',3.75,3.95)) or not all(
            abs(event['servo']-.6)<1e-6 for event in rows('servo',3.75,3.95)):
        raise RuntimeError('RC pair mixed with autonomous reverse target')
    if not all(event['mode']==SelectedControl.AUTONOMY for event in rows('selection',4.15,4.35)):
        raise RuntimeError('Valid neutral RC did not resume autonomy')
    for name,lo,hi in [('lateral_error_silence',4.85,5.05),('odometry_silence',5.95,6.15),
                       ('skew',6.95,7.15),('wrong_reference',7.35,7.55),
                       ('stale_odometry',7.85,7.95),('invalid_error',8.25,8.35),('silence',9.45,9.9)]:
        # A phase boundary can introduce a fresh counterpart that legitimately
        # pairs with the other still-valid cache. The STOP assertion belongs
        # entirely inside its fault phase, never inside that next recovery.
        if not any(phase['mode']==name and phase['start']<=lo<hi<=phase['end']
                   for phase in fixture['phases']):
            raise RuntimeError('STOP check overlaps a different fixture phase: '+name)
        if not all(event['erpm']==0 for event in rows('motor',lo,hi)) or not all(
                event['mode']==SelectedControl.STOP for event in rows('selection',lo,hi)):
            raise RuntimeError('Stanley pair failed to expire: '+name)
    for lo,hi in [(5.3,5.5),(6.35,6.55),(8.65,8.9)]:
        if not all(abs(event['erpm']+60.)<3e-6 for event in rows('motor',lo,hi)):
            raise RuntimeError('Fresh two-input recovery/artificial reverse mapping failed')
    for reason,lo,hi in [('measurement_skew',6.8,7.2),('lateral_error_reference_contract',7.2,7.6),
                          ('odometry_time_or_frame',7.6,8.),('nonfinite_lateral_error',8.,8.4)]:
        if not any(event['reason']==reason for event in rows('path_status',lo,hi)):
            raise RuntimeError('Missing diagnostic: '+reason)
    odometry={(event['stamp_ns'],event['frame']):event for event in events if event['kind']=='source_odometry'}
    errors={(event['stamp_ns'],event['frame']):event for event in events if event['kind']=='source_error'
            and event['reference_id']=='fixture_straight_corridor' and math.isfinite(event['error_m'])}
    pairs={};oldest_sources=set()
    for event in (event for event in events if event['kind']=='path_status' and event['reason']=='accepted'):
        values=event['values']
        stamp=lambda name:int(values[name+'_stamp_sec'])*1_000_000_000+int(values[name+'_stamp_nanosec'])
        os,es=stamp('odometry'),stamp('lateral_error')
        o=odometry.get((os,values['odometry_frame_id']));e=errors.get((es,values['lateral_error_frame_id']))
        if o is None or e is None or o['body']!=body or e['body']!=body or o['frame']!=parent or e['frame']!=parent:
            raise RuntimeError('Both exact original headers/body provenance lost')
        if e['point']!='software_fixture_rear_axle' or e['contract_id']!='stanley_frenet_clearance_fixed_zero_v1':
            raise RuntimeError('Declared clearance reference changed')
        if abs(os-es)>80_000_000:raise RuntimeError('Accepted input skew exceeded80ms')
        oldest_sources.add(values['oldest_source']);pairs.setdefault(min(os,es),[]).append((o,e))
    if oldest_sources!={'odometry','lateral_error'}:
        raise RuntimeError('Both oldest-source directions not audited')
    by_stamp={}
    for event in (event for event in events if event['kind']=='ackermann'):
        candidates=pairs.get(event['stamp_ns'],[])
        if not candidates or event['frame']!=body:raise RuntimeError('Ackermann not tied to oldest original header')
        matched=False
        for o,e in candidates:
            ey=-e['error_m'];speed=math.hypot(o['vx'],o['vy'])
            angle=max(-.4,min(.4,math.atan(1.2*ey/(speed+.1))-.8*ey))
            if abs(event['speed']+.6)<3e-8 and abs(event['steering']-angle)<3e-8:matched=True;break
        if not matched:raise RuntimeError('Preserved Stanley/Frenet numerical law changed')
        by_stamp.setdefault(event['stamp_ns'],[]).append(event)
    matched=0
    for event in (event for event in events if event['kind']=='actuator'):
        candidates=by_stamp.get(event['stamp_ns'],[])
        if not candidates:raise RuntimeError('Actuator lacks original two-measurement provenance')
        if event['frame']!=body or not any(abs(event['erpm']-100*command['speed'])<1e-8
                and abs(event['servo']-max(.4,min(.6,.5-.2*command['steering'])))<1e-8 for command in candidates):
            raise RuntimeError('Specific artificial Ackermann mapping changed')
        matched+=1
    if matched<80:raise RuntimeError('Insufficient two-input conversions')
    # Audit original measurement ages independently of callback arrival order.
    # The normative lease remains200ms; the observed backend may publish at
    # the next20ms tick and /clock observation is quantized at10ms.
    timeout=fixture['observation_timeout_s'];future=fixture['future_slack_s']
    tick=fixture['control_tick_s']+fixture['replay_clock_tick_s']
    if timeout!=.2 or future!=.02 or tick!=.03 or fixture['maximum_skew_s']!=.08:
        raise RuntimeError('Fixture timing differs from declared Stanley profile')
    origin=fixture['origin_ros_s']
    maximum_command_age=0.
    for event in (event for event in events if event['kind']=='ackermann'):
        candidates=pairs[event['stamp_ns']]
        # The pair audit above tied both stamps to the original bag sources.
        # No relation is inferred from DDS callback ordering across topics.
        if not any(all(-future-tick<=origin+event['t_s']-source['stamp_ns']/1e9<timeout+tick
                       for source in pair) for pair in candidates):
            raise RuntimeError('Command refreshed an expired original measurement')
        maximum_command_age=max(maximum_command_age,origin+event['t_s']-event['stamp_ns']/1e9)
    expiry_evidence={}
    for mode in ('lateral_error_silence','odometry_silence','stale_odometry','invalid_error','silence'):
        phase=next(phase for phase in fixture['phases'] if phase['mode']==mode)
        # Leave one observable tick before the next phase, whose first new
        # counterpart is allowed to reuse a still-fresh measurement.
        horizon=phase['end']-tick
        candidates=[event for event in events if event['kind']=='actuator'
                    and phase['start']-.3<=event['t_s']<horizon]
        if not candidates:raise RuntimeError('No original command before expiry: '+mode)
        oldest_stamp=max(event['stamp_ns'] for event in candidates)
        cutoff=oldest_stamp/1e9-origin+timeout
        if cutoff+tick>=horizon:raise RuntimeError('Fault phase too short for measured expiry: '+mode)
        observed=rows('motor',cutoff,horizon)
        zeros=[event for event in observed if event['erpm']==0]
        if not zeros or min(event['t_s'] for event in zeros)>cutoff+tick:
            raise RuntimeError('Motor did not stop within lease plus observed tick: '+mode)
        if any(event['erpm']!=0 for event in observed if event['t_s']>=cutoff+tick):
            raise RuntimeError('Motor prolonged original source lease: '+mode)
        first_zero=min(event['t_s'] for event in zeros)
        expiry_evidence[mode]={'oldest_original_stamp_ns':oldest_stamp,
            'source_lease_sec':timeout,'observation_tick_tolerance_sec':tick,
            'first_observed_zero_age_sec':origin+first_zero-oldest_stamp/1e9}
    return dict(signed_speed_preserved=True,clearance_inversion_both_signs=True,
        odometry_silence_expiry=True,lateral_error_silence_expiry=True,
        skew_rejected=True,wrong_reference_rejected=True,stale_odometry_rejected=True,
        invalid_error_rejected=True,fresh_recovery=True,both_original_headers=True,
        oldest_stamp_exact=True,rc_priority=True,neutral_resume=True,final_silence=True,
        matched_conversions=matched,oldest_sources=sorted(oldest_sources),
        maximum_observed_command_age_sec=maximum_command_age,expiry_evidence=expiry_evidence,
        phase_windows_checked=True)


def run(output):
    from .software_replay import run_campaign
    fixture=json.loads((share()/'data/stanley_frenet_replay.json').read_text())
    controller={'launch':'stanley_frenet_software.launch.py','subscriptions':subscriptions,
        'expected_subscribers':{
            'state/odometry':{'stanley_frenet','control_replay_observer'},
            'state/lateral_error':{'stanley_frenet','control_replay_observer'},
            'control/autonomy/ackermann':{'ackermann_adapter','control_replay_observer'},
            'control/path_status':{'control_replay_observer'},
            'control/ackermann_status':{'control_replay_observer'}},
        # The paused step delivers one source only. Neither the paired controller
        # nor its Ackermann adapter emits an observation-free heartbeat; requiring
        # their status here would deadlock before the second measure is played.
        # Graph readiness still includes every node and /clock subscriber. Actual
        # pairing/headers/expiry are checked by verify after finite playback.
        'clock_kinds':['selection','backend_status']}
    return run_campaign(output,inspect_profile(),fixture,make_bag,
                        lambda events:verify(events,fixture),controller)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args(argv);args.output.mkdir(parents=True,exist_ok=False)
    result=run(args.output.resolve());print(json.dumps(result,indent=2),flush=True)
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
