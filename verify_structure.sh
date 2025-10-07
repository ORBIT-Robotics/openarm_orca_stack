#!/bin/bash

echo "=== OPENARM ORCA STACK STRUCTURE VERIFICATION ==="
echo ""

# Check all required files exist
echo "1. Checking file existence..."
echo ""

files=(
  "assembly_description/urdf/assembly.urdf"
  "assembly_description/launch/view_robot.launch.py"
  "assembly_description/rviz/default.rviz"
  "assembly_description/CMakeLists.txt"
  "assembly_description/package.xml"
  "connectors_description/urdf/connector.urdf.xacro"
  "connectors_description/meshes/connector.stl"
  "connectors_description/CMakeLists.txt"
  "connectors_description/package.xml"
  "openarm_description/urdf/v10_dual.urdf.xacro"
  "orcahand_description/urdf/orcahand_left.urdf.xacro"
  "orcahand_description/urdf/orcahand_right.urdf.xacro"
)

all_exist=true
for file in "${files[@]}"; do
  if [ -f "$file" ]; then
    echo "✅ $file"
  else
    echo "❌ MISSING: $file"
    all_exist=false
  fi
done

echo ""
echo "2. Checking file references in assembly.urdf..."
echo ""

# Check assembly.urdf references
grep -q "v10_dual.urdf.xacro" assembly_description/urdf/assembly.urdf && echo "✅ References v10_dual.urdf.xacro" || echo "❌ Missing v10_dual.urdf.xacro reference"
grep -q "connector.urdf.xacro" assembly_description/urdf/assembly.urdf && echo "✅ References connector.urdf.xacro" || echo "❌ Missing connector.urdf.xacro reference"
grep -q "orcahand_left.urdf.xacro" assembly_description/urdf/assembly.urdf && echo "✅ References orcahand_left.urdf.xacro" || echo "❌ Missing orcahand_left.urdf.xacro reference"
grep -q "orcahand_right.urdf.xacro" assembly_description/urdf/assembly.urdf && echo "✅ References orcahand_right.urdf.xacro" || echo "❌ Missing orcahand_right.urdf.xacro reference"

echo ""
echo "3. Checking connector.urdf.xacro mesh reference..."
echo ""

grep -q "connector.stl" connectors_description/urdf/connector.urdf.xacro && echo "✅ References connector.stl" || echo "❌ Missing connector.stl reference"

echo ""
echo "4. Checking launch file references..."
echo ""

grep -q "assembly.urdf" assembly_description/launch/view_robot.launch.py && echo "✅ References assembly.urdf" || echo "❌ Missing assembly.urdf reference"
grep -q "default.rviz" assembly_description/launch/view_robot.launch.py && echo "✅ References default.rviz" || echo "❌ Missing default.rviz reference"

echo ""
echo "5. Checking CMakeLists.txt install commands..."
echo ""

grep -q "install" assembly_description/CMakeLists.txt && echo "✅ assembly_description has install commands" || echo "❌ assembly_description missing install commands"
grep -q "install" connectors_description/CMakeLists.txt && echo "✅ connectors_description has install commands" || echo "❌ connectors_description missing install commands"

echo ""
if [ "$all_exist" = true ]; then
  echo "=== ✅ ALL CRITICAL FILES EXIST ==="
else
  echo "=== ❌ SOME FILES ARE MISSING ==="
fi
echo ""