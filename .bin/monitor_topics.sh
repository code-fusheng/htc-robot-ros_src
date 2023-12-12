#!/bin/bash

# topics=$(rostopic list)
topics=(
    "/set_pcdmap_manager"
    "/ndtmapping/res"
    "/ndtmapping/control"
    "/ndtmapping/force_stop"
    "/mapping/save_map"
    "/vehicle/twist"
    "/gundam/task_control"
    "/gridmap_task_set"
    "/gridmap_task_status"
    "/pure_pursuit/task_finished"
)

output_file="all_topics_output.txt"

echo "" > "$output_file"

for topic in $topics; do
    echo "Monitoring topic: $topic" >> "$output_file"
    rostopic echo $topic | tee -a "$output_file" &
done

wait

echo "Monitoring completed. Output is stored in: $output_file"