#!/bin/bash

# Script to process all GPS path files in grt/grt/bus_2d_path
# Runs gps2tum.py and format_path.py on each file

# Input directory
GRT_DIR="Files/grt/bus_2d_path"
FLOAM_DIR="Files/F-LOAM"

# Output directory for intermediate TUM files
INTERMEDIATE_DIR="Files/grt/bus_2d_tum"

# Output directory for final formatted files
OUTPUT_DIR="Files/F-LOAM_formatted"

# Create output directories if they don't exist
mkdir -p "$INTERMEDIATE_DIR"
mkdir -p "$OUTPUT_DIR"

# Process each .txt file in the GRT directory
rm -rf "$INTERMEDIATE_DIR"/*.txt
rm -rf "$OUTPUT_DIR"/*.txt



DATASET=(ty1 wt3)
for file in "${DATASET[@]}"; do
    # Get the base filename without path
    
    # Remove .txt extension
    name="${file}_bus_grt_path"
    
    echo "Processing ${name}..."
    fname="${file}_slam_pose"
    # Step 1: Convert GPS to TUM format
    tum_file="${INTERMEDIATE_DIR}/${name}_tum.txt"
    grt_file="${GRT_DIR}/${name}.txt"
    floam_file="${FLOAM_DIR}/${fname}.txt"
    echo "  1/2: Converting GPS to TUM format..."
    python3 gps2tum.py "$grt_file" --f-input "$floam_file" -o "$tum_file"
    
    if [ $? -ne 0 ]; then
        echo "  Error: gps2tum.py failed for $grt_file"
        continue
    fi
done
DATASET2=(ty1 wt3)
for file in "${DATASET2[@]}"; do
    # Get the base filename without path

    # Remove .txt extension
    fname="${file}_slam_pose"
    echo "Processing $basename..."
    # Step 2: Format the TUM file
    formatted_file="${OUTPUT_DIR}/${fname}_formatted.txt"
    floam_file="${FLOAM_DIR}/${fname}.txt"
    source="${INTERMEDIATE_DIR}/${file}_bus_grt_path_tum.txt"
    echo "  2/2: Formatting TUM file..."
    python3 format_path.py "$floam_file" --source "$source"  -o "$formatted_file"
    
    if [ $? -ne 0 ]; then
        echo "  Error: format_path.py failed for $file"
        continue
    fi
    
    echo "  ✓ Completed: $formatted_file"
done

rm -rf imgs/*.png
for ds in "${DATASET[@]}"; do
    echo "Processing dataset: $ds"
    grt_file="${INTERMEDIATE_DIR}/${ds}_bus_grt_path_tum.txt"
    floam_file="${OUTPUT_DIR}/${ds}_slam_pose_formatted.txt"
    
    evo_traj tum "$grt_file" "$floam_file" --plot_mode=xy --save_plot "imgs/${ds}_comparison.png"
    
    echo "  ✓ Completed: $ds"
done
grt_file="Files/grt/gps_path/hh1_gps_path_grt.txt"
floam_file="${FLOAM_DIR}/hh1_slam_pose.txt"
formatted_file="${OUTPUT_DIR}/hh1_slam_pose_formatted.txt"
python3 format_path.py "$floam_file" --source "$grt_file"  -o "$formatted_file"
evo_traj tum "$grt_file" "$formatted_file" --plot_mode=xy --save_plot "imgs/hh1_comparison.png" 
    
echo "  ✓ Completed: hh1"

rm -rf imgs/*_speeds.png imgs/*_comparison_xyz.png imgs/*_comparison_rpy.png


echo ""
echo "All files processed!"
echo "Intermediate TUM files: $INTERMEDIATE_DIR"
echo "Final formatted files: $OUTPUT_DIR"
