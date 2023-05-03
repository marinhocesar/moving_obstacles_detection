class GlobalConfig:
    SEGMENTATION_THRESHOLD = 0.1
    EXCLUSION_THRESHOLD = 25  # degrees
    MAX_SKIPPED = -1  # maximum skipped points
    MIN_ANGLE = 40
    MAX_ANGLE = 140
    ECCENTRICITY_THRESHOLD_COEFFICIENT = 1 / 5
    MAX_RANGE = 0.75
    SHOULD_JOIN = False
    MAX_ALLOWED_MEASURE_DIFF = 0.1
    SHOULD_RENDER_FIT_CURVE = False

    # SEED_SEGMENT DETECTION
    P_MIN = 10  # minimum number of laser points contained in an extracted line segment
    S_NUM = 6  # number of points in a seed-segment
    DISTANCE_THRESHOLD_POINT_LINE = 0.03  # meters
    DISTANCE_THRESHOLD_POINT_POINT = 0.1  # meters

    # rounding factor used when mapping lidar data to a python dict using the truncated angle as a key0
    ROUNDING_FACTOR = 10**10

    # number of adjacent points to be checked while wanting to determine if a point is apparent or real significant
    NEIGHBORHOOD_CHECK_FOR_APPARENT_SIGNIFICANT = 10

    DIFFERENCE_FOR_DISPLACEMENT = 1
    BUFFER_SIZE = 5
    DISPLACEMENT_THRESHOLD = 0.03
    ANIMATION_INTERVAL = 50  # Delay between frames in milliseconds.

    VECTOR_SCALING = 1

    MAX_MATCHING_DISTANCE = 0.25
    SPEED_THRESHOLD = 0.5  # m/s
