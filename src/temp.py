import laspy

file_path = "/Volumes/T7 Shield/TUD/2024_C_44HZ1.LAZ"

with laspy.open(file_path) as laz_file:
    header = laz_file.header
    point_format = header.point_format
    for dimension_name in point_format.dimension_names:
        dimension_info = point_format.dimension_by_name(dimension_name)
        print(dimension_info)