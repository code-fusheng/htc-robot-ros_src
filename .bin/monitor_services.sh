#!/bin/bash

# services=$(rosservice list)
services=(
    "/online_mapping"
    "/fuunction_switch/ndt_localization"
    "/ndt_localization/service_solution_status"
    "/function_switch/pcd_map_manager"
    "/motor_control"
    "/function_switch/global_planning"
    "/lb_cloud_service/set_target"
    "/set_ppt_target"
    "/avoid_type"
)

output_file="all_services_output.txt"

echo "" > "$output_file"

for service in $services; do
    echo "Monitoring service: $service" >> "$output_file"
    rosservice echo $service | tee -a "$output_file" &
done

wait

echo "Monitoring completed. Output is stored in: $output_file"