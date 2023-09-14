import numpy as np
import math


def reward_function(params):
    reward = 0.001

    if not params['all_wheels_on_track']:
        return reward

    heading = params['heading']
    track_width = params['track_width']
    waypoints = params['waypoints']
    is_left_of_center = params['is_left_of_center']
    closest_waypoints = params['closest_waypoints']
    speed = params['speed']
    distance_from_center = params['distance_from_center']

    # SPEED CONSTANTS
    slow = 1.5
    fast = 2.5
    blazing = 3.5

    # TURN GRADIANT CONSTANTS
    slight = 8
    sharp = 15
    hairpin = 35

    # waypoints
    prev_point_index = closest_waypoints[0]
    prev_point = getNextWaypoint(waypoints, prev_point_index, 0)
    next_point_0 = getNextWaypoint(waypoints, prev_point_index, 1)
    next_point_1 = getNextWaypoint(waypoints, prev_point_index, 2)
    next_point_2 = getNextWaypoint(waypoints, prev_point_index, 3)

    # track section headings/angles (relative to previos track section angle)
    current_track_section_heading = getSectionHeading(prev_point, next_point_0)
    next_track_section_0_heading = getSectionHeading(
        next_point_0, next_point_1)
    next_track_section_1_heading = getSectionHeading(
        next_point_1, next_point_2)

    # car heading relative to track sections
    car_heading_relative_to_current_track_section = calculateHeadingRelativeToAngle(
        heading, current_track_section_heading)
    car_heading_relative_to_next_track_section_0 = calculateHeadingRelativeToAngle(
        heading, next_track_section_0_heading)
    car_heading_relative_to_next_track_section_1 = calculateHeadingRelativeToAngle(
        heading, next_track_section_1_heading)

    # speed bonuses relative to track section headings only
    if current_track_section_heading == next_track_section_0_heading and speed >= fast:
        reward += 10
        if next_track_section_0_heading == next_track_section_1_heading and speed >= blazing:
            reward += 10

    # speed bonuses relative to track car headings only
    if car_heading_relative_to_current_track_section < slight and speed >= slow:
        reward += 10

    if car_heading_relative_to_next_track_section_0 < slight and current_track_section_heading == next_track_section_0_heading and speed >= blazing:
        reward += 10

    # track position bonuses only
    if next_track_section_0_heading > 0:
        # approaching, or in the middle of a right turn
        if next_track_section_0_heading < next_track_section_1_heading and is_left_of_center == True:
            # car is on the outside of the track approaching a right turn (prior to apex)
            reward += 10
        elif next_track_section_0_heading > next_track_section_1_heading and is_left_of_center == False:
            # car is on the outside of the track as it exits a right turn (after apex)
            reward += 10

    if next_track_section_0_heading < 0:
        # approaching, or in the middle of a left turn
        if next_track_section_0_heading > next_track_section_1_heading and is_left_of_center == False:
            # car is on the outside of the track approaching a left turn (prior to apex)
            reward += 10
        elif next_track_section_0_heading < next_track_section_1_heading and is_left_of_center == True:
            # car is on the outside of the track as it exits a right turn (after apex)
            reward += 10

    # SLOW basic line tracing (small)bonus to get training off on the right foot
    marker_1 = 0.1 * track_width
    marker_2 = 0.25 * track_width
    marker_3 = 0.45 * track_width

    if distance_from_center <= marker_1:
        reward += 6.0
    elif distance_from_center <= marker_2:
        reward += 3
    elif distance_from_center <= marker_3:
        reward += 1

    # reward the vehicle
    return reward


# return waypoint at nextIndex after currentWaypointIndex
def getNextWaypoint(waypoints, currentWaypointIndex, nextIndex):
    wlen = len(waypoints)
    if currentWaypointIndex + nextIndex >= wlen:
        return waypoints[currentWaypointIndex + nextIndex - wlen]
    else:
        return waypoints[currentWaypointIndex + nextIndex]

# gets the angle between the two points relative to the coordinate grid


def getSectionHeading(startPoint, endPoint):
    radians = math.atan2(endPoint[1] - startPoint[1],
                         endPoint[0] - startPoint[0])
    degrees = math.degrees(radians)
    return degrees

# negative indicates left turn, positive indicates right turn


def calculateHeadingRelativeToAngle(headingAngle, relativeBaseAngle):
    return (headingAngle - relativeBaseAngle + 180) % 360 - 180
