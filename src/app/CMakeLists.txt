add_executable(run_frontend_offline
        run_frontend_offline.cc
)

target_link_libraries(run_frontend_offline
        ${PROJECT_NAME}.libs
        ${third_party_libs}
)

add_executable(run_loop_offline
        run_loop_offline.cc
)

target_link_libraries(run_loop_offline
        ${PROJECT_NAME}.libs
        ${third_party_libs}
)

add_executable(run_slam_offline
        run_slam_offline.cc
)

target_link_libraries(run_slam_offline
        ${PROJECT_NAME}.libs
        ${third_party_libs}
)

add_executable(run_slam_online
        run_slam_online.cc
)

target_link_libraries(run_slam_online
        ${PROJECT_NAME}.libs
        ${third_party_libs}
)

add_executable(run_loc_offline
        run_loc_offline.cc
)

target_link_libraries(run_loc_offline
        ${PROJECT_NAME}.libs
        ${third_party_libs}
)

add_executable(run_loc_online
        run_loc_online.cc
)

target_link_libraries(run_loc_online
        ${PROJECT_NAME}.libs
        ${third_party_libs}
)

add_executable(test_ui
        test_ui.cc
)

target_link_libraries(test_ui
        ${PROJECT_NAME}.libs
        ${third_party_libs}
)

# ------- tf transformer -------
add_executable(tf_transformer
        tf_transformer.cc
)

ament_target_dependencies(tf_transformer
        rclcpp
        sensor_msgs
        geometry_msgs
        tf2
        tf2_ros
        tf2_geometry_msgs
        tf2_sensor_msgs
)

add_dependencies(tf_transformer
        livox_interfaces2__rosidl_typesupport_cpp
)

target_link_libraries(tf_transformer
        ${PROJECT_NAME}.libs
        ${third_party_libs}
)

# ------- static_tf_broadcaster ------
add_executable(static_tf_broadcaster
        static_tf_broadcaster.cc
)

ament_target_dependencies(static_tf_broadcaster
        rclcpp
        geometry_msgs
        tf2_ros
)

target_link_libraries(static_tf_broadcaster
        ${PROJECT_NAME}.libs
        ${third_party_libs}
)


install(TARGETS static_tf_broadcaster tf_transformer run_slam_offline run_slam_online run_loc_offline run_loc_online
        DESTINATION lib/${PROJECT_NAME})
