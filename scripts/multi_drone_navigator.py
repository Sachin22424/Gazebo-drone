#!/usr/bin/env python3

import os
import random
import threading
import yaml
from datetime import datetime
from math import atan2, exp, pi, sqrt

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


def clamp(value, minimum=0.0, maximum=1.0):
    return max(minimum, min(maximum, value))


def normalize_angle(angle):
    while angle > pi:
        angle -= 2 * pi
    while angle < -pi:
        angle += 2 * pi
    return angle


def yaw_from_quaternion(quat):
    siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
    cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
    return atan2(siny_cosp, cosy_cosp)


class MissionAggregator:
    def __init__(self):
        self._lock = threading.Lock()
        self._results = []

    def add_result(self, entry):
        with self._lock:
            self._results.append(entry)

    def get_results(self):
        with self._lock:
            return list(self._results)


class RiskMarkerPublisher:
    def __init__(self, frame_id='world'):
        self._publisher = rospy.Publisher('risk_markers', MarkerArray, queue_size=10, latch=True)
        self._frame_id = frame_id
        self._lock = threading.Lock()
        self._markers = {}

    def _publish(self):
        array = MarkerArray()
        array.markers = list(self._markers.values())
        self._publisher.publish(array)

    def _color_for_probability(self, probability):
        probability = clamp(probability, 0.0, 1.0)
        color = ColorRGBA()
        color.r = probability
        color.g = 1.0 - probability
        color.b = 0.0
        color.a = 1.0
        return color

    def update_area(self, marker_id, area_name, area_cfg, model_probability, measured_probability):
        marker = Marker()
        marker.header.frame_id = self._frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = 'drought_risk'
        marker.id = 1000 + marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = area_cfg.get('x', 0.0)
        marker.pose.position.y = area_cfg.get('y', 0.0)
        marker.pose.position.z = area_cfg.get('z', 2.0) + 2.0
        marker.scale.z = 0.8
        marker.color = self._color_for_probability(model_probability)
        marker.text = (
            f"Area {area_name}\n"
            f"Drone {marker_id}\n"
            f"Model: {model_probability*100:.1f}%\n"
            f"Onboard: {measured_probability*100:.1f}%"
        )

        with self._lock:
            self._markers[f'area-{area_name}'] = marker
            self._publish()

    def update_reserve(self, marker_id, start_pos, measured_probability):
        marker = Marker()
        marker.header.frame_id = self._frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = 'drought_reserve'
        marker.id = 2000 + marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = start_pos.get('x', 0.0)
        marker.pose.position.y = start_pos.get('y', 0.0)
        marker.pose.position.z = start_pos.get('z', 2.0) + 2.0
        marker.scale.z = 0.7
        color = ColorRGBA()
        color.r = 0.3
        color.g = 0.6
        color.b = 1.0
        color.a = 1.0
        marker.color = color
        marker.text = (
            f"Drone {marker_id}\n"
            f"Reserve\n"
            f"Onboard: {measured_probability*100:.1f}%"
        )

        with self._lock:
            self._markers[f'reserve-{marker_id}'] = marker
            self._publish()


def analyze_drought_risk(area_name, area_config):
    history = area_config.get('drought_history', [])

    if not history:
        return {
            'probability': 0.3,
            'avg_rainfall_deficit': 0.0,
            'avg_soil_moisture_deficit': 0.0,
            'avg_vegetation_stress': 0.0,
            'avg_heatwave_intensity': 0.0,
            'historical_drought_rate': 0.0,
            'trend_factor': 0.0,
            'years': []
        }

    rainfall_deficits = [clamp(year['rainfall_deficit']) for year in history]
    soil_moisture_deficits = [clamp(1.0 - year['soil_moisture_index']) for year in history]
    vegetation_stress = [clamp(1.0 - year['veg_health_index']) for year in history]
    heatwave_intensity = [clamp(year.get('heatwave_days', 0) / 30.0) for year in history]
    drought_flags = [1.0 if year.get('drought_declared', False) else 0.0 for year in history]

    count = float(len(history))
    avg_rainfall = sum(rainfall_deficits) / count
    avg_soil_moisture = sum(soil_moisture_deficits) / count
    avg_vegetation = sum(vegetation_stress) / count
    avg_heatwave = sum(heatwave_intensity) / count
    drought_rate = sum(drought_flags) / count

    rainfall_trend = rainfall_deficits[-1] - rainfall_deficits[0]
    soil_trend = soil_moisture_deficits[-1] - soil_moisture_deficits[0]
    vegetation_trend = vegetation_stress[-1] - vegetation_stress[0]
    heat_trend = heatwave_intensity[-1] - heatwave_intensity[0]

    trend_factor = clamp(0.35 * rainfall_trend + 0.25 * soil_trend + 0.2 * vegetation_trend + 0.2 * heat_trend, -0.3, 0.3)

    composite_score = (
        0.42 * avg_rainfall +
        0.22 * avg_soil_moisture +
        0.16 * avg_vegetation +
        0.12 * avg_heatwave +
        0.08 * drought_rate
    )

    composite_score = clamp(composite_score + trend_factor, 0.0, 1.0)

    probability = 1.0 / (1.0 + exp(-6.0 * (composite_score - 0.5)))
    probability = clamp(probability + (trend_factor * 0.4), 0.05, 0.95)

    return {
        'probability': probability,
        'avg_rainfall_deficit': avg_rainfall,
        'avg_soil_moisture_deficit': avg_soil_moisture,
        'avg_vegetation_stress': avg_vegetation,
        'avg_heatwave_intensity': avg_heatwave,
        'historical_drought_rate': drought_rate,
        'trend_factor': trend_factor,
        'years': history
    }


def build_allocation_report(log_path, areas_cfg, area_profiles, allocation, full_plan, mission_results=None):
    os.makedirs(os.path.dirname(log_path), exist_ok=True)

    timestamp = datetime.utcnow().strftime('%Y-%m-%d %H:%M:%S')

    lines = []
    lines.append('MULTI-DRONE FARMLAND DROUGHT ALLOCATION REPORT')
    lines.append(f'Generated (UTC): {timestamp}')
    lines.append('')
    lines.append('Drought probability model:')
    lines.append('  * Weighted rainfall deficit, soil moisture deficit, vegetation stress, and heatwave intensity')
    lines.append('  * Recent trend bonus captures worsening climatic signals')
    lines.append('  * Logistic mapping keeps probabilities within 5%-95% confidence range')
    lines.append('')
    lines.append('Area drought risk summary:')
    lines.append('Area   Farm Name           P(drought)  RainDef  SoilDef  VegStress  HeatIdx  DroughtFreq  Trend')
    lines.append('-----  ------------------  ----------  -------  -------  ---------  -------  -----------  -----')

    ordered_areas = sorted(areas_cfg.keys(), key=lambda name: area_profiles[name]['probability'], reverse=True)
    for name in ordered_areas:
        profile = area_profiles[name]
        area_cfg = areas_cfg[name]
        farm_name = area_cfg.get('name', name)
        lines.append(
            f"{name:<5}  {farm_name:<18}  {profile['probability']*100:>9.2f}%  "
            f"{profile['avg_rainfall_deficit']:>6.2f}  {profile['avg_soil_moisture_deficit']:>6.2f}  "
            f"{profile['avg_vegetation_stress']:>7.2f}  {profile['avg_heatwave_intensity']:>6.2f}  "
            f"{profile['historical_drought_rate']:>10.2f}  {profile['trend_factor']:>5.2f}"
        )

    lines.append('')
    lines.append('Drone-to-area allocation:')
    lines.append('Drone  Role       Assignment (segment)              Risk   Notes')
    lines.append('-----  ---------  -------------------------------  ------  ---------------------------')

    for idx, plan in enumerate(full_plan):
        if plan['role'] == 'explorer':
            area_name = plan['area']
            area_cfg = areas_cfg[area_name]
            farm_name = area_cfg.get('name', area_name)
            segment = f"{plan['group_index'] + 1}/{plan['group_size']}"
            risk = area_profiles[area_name]['probability'] * 100.0
            notes = f"{farm_name} ({area_cfg.get('color', 'n/a')})"
            lines.append(f"{idx:<5}  explorer  {area_name:<7} {farm_name:<20} {segment:<9}  {risk:>6.1f}%  {notes}")
        else:
            lines.append(f"{idx:<5}  backup    staging-area                 n/a       ---    Holding position")

    mission_results = mission_results or []
    if mission_results:
        lines.append('')
        lines.append('Mission risk observations:')
        lines.append('Drone  Farm Name           Actual  Onboard  Error   Inside  Bounds  Final (x,y)     Status   Notes')
        lines.append('-----  ------------------  -------  -------  -------  ------  ------  --------------  -------  ---------------------------')

        for result in sorted(mission_results, key=lambda entry: entry['drone_id']):
            actual = result.get('actual_probability')
            onboard = result.get('measured_probability')
            error_pct = result.get('error_pct')
            boundary_events = result.get('boundary_events', 0)
            final_pos = result.get('final_position', '--')
            status = result.get('status', 'n/a')
            notes = result.get('notes', '')
            inside = result.get('within_bounds')

            actual_str = f"{actual * 100:6.1f}%" if actual is not None else '  --  '
            onboard_str = f"{onboard * 100:6.1f}%" if onboard is not None else '  --  '
            error_str = f"{error_pct:+6.2f}%" if error_pct is not None else '  --  '
            inside_str = ' yes ' if inside is True else (' no  ' if inside is False else '  -- ')

            lines.append(
                f"{result['drone_id']:<5}  {result.get('farm_name', 'n/a'):<18}  "
                f"{actual_str:<7}  {onboard_str:<7}  {error_str:<7}  "
                f"{inside_str:<6}  {boundary_events:<6}  {final_pos:<14}  {status:<7}  {notes}"
            )

    lines.append('')
    with open(log_path, 'w') as log_file:
        log_file.write('\n'.join(lines))


class ExplorerDrone:
    def __init__(self, drone_id, area_name, area_cfg, risk_profile, measurement_noise, boundary_margin, aggregator, marker_manager=None):
        self.drone_id = drone_id
        self.area_name = area_name
        self.area_cfg = area_cfg
        self.risk_profile = risk_profile
        self.measurement_noise = abs(measurement_noise)
        self.boundary_margin = max(0.2, boundary_margin)
        self.aggregator = aggregator
        self.marker_manager = marker_manager

        self.current_pose = None
        self.target_reached = False
        self.result_recorded = False
        self.boundary_events = 0
        self.arrival_announced = False

        self.target_x = area_cfg['x']
        self.target_y = area_cfg['y']
        self.target_z = area_cfg.get('z', 2.0)
        self.farm_name = area_cfg.get('name', area_name)

        half_size = max(1.0, area_cfg.get('size', 8.0) / 2.0)
        self.min_x = self.target_x - half_size + self.boundary_margin
        self.max_x = self.target_x + half_size - self.boundary_margin
        self.min_y = self.target_y - half_size + self.boundary_margin
        self.max_y = self.target_y + half_size - self.boundary_margin

        self.actual_probability = risk_profile.get('probability', 0.5)
        noise = random.uniform(-self.measurement_noise, self.measurement_noise)
        self.measured_probability = clamp(self.actual_probability * (1.0 + noise))
        self.risk_error_pct = (self.measured_probability - self.actual_probability) * 100.0
        self.notes = [
            f"Risk model {self.actual_probability*100:.1f}%",
            f"Onboard sensor {self.measured_probability*100:.1f}%"
        ]

        if self.marker_manager:
            self.marker_manager.update_area(
                self.drone_id,
                self.area_name,
                self.area_cfg,
                self.actual_probability,
                self.measured_probability
            )

        self.cmd_vel_pub = rospy.Publisher(f'/drone_{drone_id}/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber(f'/drone_{drone_id}/odom', Odometry, self.odom_callback)

        self.linear_gain = 0.9
        self.max_linear_vel = 2.5
        self.angular_gain = 1.8
        self.max_angular_vel = 1.4

        rospy.loginfo(
            f"[Drone {drone_id}] Explorer assigned to {self.farm_name} ({area_name}) | "
            f"model risk {self.actual_probability*100:.1f}%"
        )
        rospy.loginfo(
            f"[Drone {drone_id}] Onboard drought estimate {self.measured_probability*100:.1f}% "
            f"(error {self.risk_error_pct:+.2f}%)"
        )

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def stop_motion(self):
        self.cmd_vel_pub.publish(Twist())

    def is_within_bounds(self):
        if self.current_pose is None:
            return False
        x = self.current_pose.position.x
        y = self.current_pose.position.y
        return self.min_x <= x <= self.max_x and self.min_y <= y <= self.max_y

    def record_result(self, status, note=None):
        if self.result_recorded:
            return

        if note:
            self.notes.append(note)
        if self.boundary_events:
            label = 'corrections' if self.boundary_events > 1 else 'correction'
            self.notes.append(f"Boundary {label}: {self.boundary_events}")

        inside_bounds = False
        if self.current_pose is not None:
            final_pos = f"({self.current_pose.position.x:.2f}, {self.current_pose.position.y:.2f})"
            inside_bounds = self.is_within_bounds()
        else:
            final_pos = f"({self.target_x:.2f}, {self.target_y:.2f})"
            inside_bounds = self.target_reached

        summary = {
            'drone_id': self.drone_id,
            'farm_name': self.farm_name,
            'area_name': self.area_name,
            'actual_probability': self.actual_probability,
            'measured_probability': self.measured_probability,
            'error_pct': self.risk_error_pct,
            'boundary_events': self.boundary_events,
            'final_position': final_pos,
            'status': status,
            'within_bounds': inside_bounds,
            'notes': '; '.join(self.notes)
        }

        self.aggregator.add_result(summary)
        self.result_recorded = True
        if self.marker_manager:
            self.marker_manager.update_area(
                self.drone_id,
                self.area_name,
                self.area_cfg,
                self.actual_probability,
                self.measured_probability
            )

    def drive_toward(self, target_x, target_y, tight=False):
        if self.current_pose is None:
            return float('inf')

        dx = target_x - self.current_pose.position.x
        dy = target_y - self.current_pose.position.y
        distance = sqrt(dx * dx + dy * dy)

        desired_yaw = atan2(dy, dx)
        current_yaw = yaw_from_quaternion(self.current_pose.orientation)
        angle_error = normalize_angle(desired_yaw - current_yaw)

        cmd = Twist()
        max_linear = 1.0 if tight else self.max_linear_vel
        cmd.linear.x = min(max_linear, max(0.0, distance * (0.6 if tight else self.linear_gain)))
        if abs(angle_error) > 1.1:
            cmd.linear.x = 0.0

        cmd.angular.z = max(
            -self.max_angular_vel,
            min(self.max_angular_vel, angle_error * self.angular_gain)
        )

        self.cmd_vel_pub.publish(cmd)
        return distance

    def navigate(self):
        rate = rospy.Rate(10)
        arrival_radius = 0.6
        hold_seconds = 3.0
        hold_start = None

        try:
            while not rospy.is_shutdown():
                if self.current_pose is None:
                    rate.sleep()
                    continue

                dx = self.target_x - self.current_pose.position.x
                dy = self.target_y - self.current_pose.position.y
                distance = sqrt(dx * dx + dy * dy)
                inside_bounds = self.is_within_bounds()

                if not self.target_reached:
                    if distance < arrival_radius or inside_bounds:
                        self.target_reached = True
                        hold_start = rospy.Time.now()
                        self.stop_motion()
                        if not self.arrival_announced:
                            rospy.loginfo(
                                f"✓ Drone {self.drone_id} drought assessment at {self.farm_name} | "
                                f"model {self.actual_probability*100:.1f}% vs onboard {self.measured_probability*100:.1f}% "
                                f"(error {self.risk_error_pct:+.2f}%)"
                            )
                            self.arrival_announced = True
                        rate.sleep()
                        continue

                    self.drive_toward(self.target_x, self.target_y)

                else:
                    if not inside_bounds:
                        self.boundary_events += 1
                        rospy.logwarn(
                            f"[Drone {self.drone_id}] Boundary correction #{self.boundary_events} at {self.farm_name}"
                        )
                        hold_start = None
                        self.drive_toward(self.target_x, self.target_y, tight=True)
                    elif distance > 0.25:
                        hold_start = None
                        self.drive_toward(self.target_x, self.target_y, tight=True)
                    else:
                        self.stop_motion()
                        if hold_start is None:
                            hold_start = rospy.Time.now()

                    if hold_start and (rospy.Time.now() - hold_start).to_sec() > hold_seconds:
                        self.record_result('holding', note='Holding within farm bounds')
                        rospy.loginfo(f"Drone {self.drone_id} remains inside {self.farm_name} bounds")
                        break

                rate.sleep()
        finally:
            self.stop_motion()
            if not self.result_recorded:
                status = 'aborted' if not self.target_reached else 'holding'
                note = 'Navigation interrupted before reaching area' if status == 'aborted' else 'Hold interrupted during shutdown'
                self.record_result(status, note=note)


class ReserveDrone:
    def __init__(self, drone_id, start_pos, aggregator, measurement_noise, marker_manager=None):
        self.drone_id = drone_id
        self.start_pos = start_pos
        self.aggregator = aggregator
        self.measurement_noise = abs(measurement_noise)
        self.marker_manager = marker_manager

        self.current_pose = None
        self.result_recorded = False

        self.actual_probability = 0.0
        noise = random.uniform(-self.measurement_noise, self.measurement_noise)
        self.measured_probability = clamp(self.actual_probability * (1.0 + noise))
        self.risk_error_pct = (self.measured_probability - self.actual_probability) * 100.0

        self.cmd_vel_pub = rospy.Publisher(f'/drone_{drone_id}/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber(f'/drone_{drone_id}/odom', Odometry, self.odom_callback)

        rospy.loginfo(
            f"[Drone {drone_id}] Reserve holding at spawn | onboard risk {self.measured_probability*100:.1f}%"
        )

        if self.marker_manager:
            self.marker_manager.update_reserve(self.drone_id, self.start_pos, self.measured_probability)

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def stop_motion(self):
        self.cmd_vel_pub.publish(Twist())

    def record_result(self):
        if self.result_recorded:
            return

        if self.current_pose is not None:
            final_pos = f"({self.current_pose.position.x:.2f}, {self.current_pose.position.y:.2f})"
        else:
            final_pos = f"({self.start_pos['x']:.2f}, {self.start_pos['y']:.2f})"

        summary = {
            'drone_id': self.drone_id,
            'farm_name': 'Staging Pad',
            'area_name': 'staging',
            'actual_probability': self.actual_probability,
            'measured_probability': self.measured_probability,
            'error_pct': self.risk_error_pct,
            'boundary_events': 0,
            'final_position': final_pos,
            'status': 'reserve',
            'within_bounds': True,
            'notes': 'Idle drought reserve drone'
        }

        self.aggregator.add_result(summary)
        self.result_recorded = True
        if self.marker_manager:
            self.marker_manager.update_reserve(self.drone_id, self.start_pos, self.measured_probability)

    def hold_position(self):
        rate = rospy.Rate(5)
        stable_cycles = 0

        try:
            while not rospy.is_shutdown():
                if self.current_pose is None:
                    rate.sleep()
                    continue

                dx = self.start_pos['x'] - self.current_pose.position.x
                dy = self.start_pos['y'] - self.current_pose.position.y
                distance = sqrt(dx * dx + dy * dy)

                cmd = Twist()
                if distance > 0.25:
                    desired_yaw = atan2(dy, dx)
                    current_yaw = yaw_from_quaternion(self.current_pose.orientation)
                    angle_error = normalize_angle(desired_yaw - current_yaw)

                    cmd.linear.x = min(0.6, max(0.0, distance * 0.4))
                    if abs(angle_error) > 1.2:
                        cmd.linear.x = 0.0
                    cmd.angular.z = max(-1.0, min(1.0, angle_error * 1.6))
                    stable_cycles = 0
                else:
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.0
                    stable_cycles += 1

                self.cmd_vel_pub.publish(cmd)

                if stable_cycles >= 12:
                    rospy.loginfo(f"Drone {self.drone_id} secured at spawn pad")
                    self.record_result()
                    break

                rate.sleep()
        finally:
            self.stop_motion()
            if not self.result_recorded:
                self.record_result()


def main():
    rospy.init_node('multi_drone_navigator')

    config_path = rospy.get_param('~config_path', os.path.join(os.path.dirname(__file__), '../config/areas.yaml'))

    try:
        with open(config_path, 'r') as cfg_file:
            config = yaml.safe_load(cfg_file)
    except Exception as exc:
        rospy.logerr(f"Failed to load config: {exc}")
        return

    num_drones = config['num_drones']
    areas = config['areas']
    start_pos = config.get('start_position', {'x': 0.0, 'y': 0.0, 'z': 2.0})
    allocation_cfg = config.get('allocation', {})

    measurement_noise = allocation_cfg.get('measurement_noise', 0.12)
    boundary_margin = allocation_cfg.get('boundary_soft_margin', 0.4)
    idle_measurement_noise = allocation_cfg.get('idle_measurement_noise', 0.05)

    area_profiles = {name: analyze_drought_risk(name, cfg) for name, cfg in areas.items()}

    ordered_areas = sorted(areas.keys(), key=lambda name: area_profiles[name]['probability'], reverse=True)
    active_assignments = min(num_drones, len(ordered_areas))

    explorer_plan = []
    allocation_counts = {name: 0 for name in areas.keys()}

    for idx in range(active_assignments):
        area_name = ordered_areas[idx]
        explorer_plan.append({
            'role': 'explorer',
            'area': area_name,
            'group_index': 0,
            'group_size': 1,
            'probability': area_profiles[area_name]['probability']
        })
        allocation_counts[area_name] += 1

    reserve_count = max(0, num_drones - len(explorer_plan))
    full_plan = explorer_plan + [{'role': 'backup'} for _ in range(reserve_count)]
    if len(full_plan) < num_drones:
        full_plan.extend([{'role': 'backup'} for _ in range(num_drones - len(full_plan))])
    full_plan = full_plan[:num_drones]

    marker_manager = RiskMarkerPublisher(allocation_cfg.get('world_frame', 'world'))
    aggregator = MissionAggregator()

    rospy.loginfo("=" * 60)
    rospy.loginfo("         MULTI-DRONE DROUGHT NAVIGATION")
    rospy.loginfo("=" * 60)
    rospy.loginfo(f"Total drones: {num_drones}")
    rospy.loginfo(f"Exploration areas: {len(areas)}")
    rospy.loginfo("")
    rospy.loginfo("Drought risk ranking:")
    for name in ordered_areas:
        profile = area_profiles[name]
        area_cfg = areas[name]
        rospy.loginfo(
            f"  {area_cfg.get('name', name):<18} (Area {name}) | "
            f"P(drought)={profile['probability']*100:.1f}% | Assigned drones={allocation_counts.get(name, 0)}"
        )
    rospy.loginfo(f"Reserve drones: {reserve_count}")
    rospy.loginfo("=" * 60)

    report_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'logs', 'drought_allocation.log'))
    build_allocation_report(report_path, areas, area_profiles, allocation_counts, full_plan, mission_results=[])
    rospy.loginfo(f"Initial allocation report written to {report_path}")

    rospy.loginfo("Waiting for simulation to stabilise...")
    rospy.sleep(5.0)
    rospy.loginfo("Waiting for odometry data...")
    rospy.sleep(3.0)

    explorers = []
    reserves = []

    for drone_id in range(num_drones):
        plan = full_plan[drone_id]
        if plan['role'] == 'explorer':
            area_name = plan['area']
            explorer = ExplorerDrone(
                drone_id,
                area_name,
                areas[area_name],
                area_profiles[area_name],
                measurement_noise,
                boundary_margin,
                aggregator,
                marker_manager
            )
            explorers.append(explorer)
        else:
            reserve = ReserveDrone(
                drone_id,
                start_pos,
                aggregator,
                idle_measurement_noise,
                marker_manager
            )
            reserves.append(reserve)

    rospy.loginfo(f"Active explorers: {len(explorers)} | Reserve drones: {len(reserves)}")
    rospy.loginfo("Launching navigation threads...")

    threads = []
    for explorer in explorers:
        thread = threading.Thread(target=explorer.navigate)
        thread.daemon = True
        thread.start()
        threads.append(thread)

    for reserve in reserves:
        thread = threading.Thread(target=reserve.hold_position)
        thread.daemon = True
        thread.start()
        threads.append(thread)

    for thread in threads:
        thread.join()

    rospy.loginfo("=" * 60)
    rospy.loginfo("All assignments processed. Drones are stable inside their zones or on standby.")
    rospy.loginfo("=" * 60)

    mission_results = aggregator.get_results()
    build_allocation_report(report_path, areas, area_profiles, allocation_counts, full_plan, mission_results=mission_results)
    rospy.loginfo(f"Mission report updated with {len(mission_results)} entries")

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except Exception as exc:
        rospy.logerr(f"Error: {exc}")

# roslaunch multi_drone_sim multi_drone_sim.launch
# source /home/sachin/catkin_ws/devel/setup.bash && roslaunch multi_drone_sim multi_drone_sim.launch