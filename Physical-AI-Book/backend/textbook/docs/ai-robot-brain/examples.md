---
title: "Module 3: Examples"
slug: /ai-robot-brain/examples
sidebar_label: "Examples"
sidebar_position: 7
toc: true
description: "Hands-on code examples for Isaac Sim scene setup, Replicator API for synthetic data, Isaac ROS vSLAM integration, and TensorRT model optimization."
---

# Module 3: GPU-Accelerated Perception & Isaac - Examples

This page provides **practical, annotated examples** to complement the conceptual material in Module 3. Each example includes complete code, configuration files, and step-by-step instructions for hands-on implementation with NVIDIA Isaac Sim, Isaac ROS, and GPU-accelerated perception pipelines.

---

## Example 1: Isaac Sim Warehouse Scene Setup with Python API

This example demonstrates how to programmatically create a complete warehouse environment using the Isaac Sim Python API with PBR materials, HDR lighting, and robot spawning.

### Scenario

Create a 30 m × 40 m warehouse scene with:
- Realistic floor with PBR materials (concrete)
- Warehouse shelving units (4 rows, 10 units each)
- HDR dome lighting (indoor warehouse environment)
- Pallet stacks and boxes for navigation testing
- Humanoid robot spawn point with cameras

### Python Script: `create_warehouse_scene.py`

```python
#!/usr/bin/env python3
"""
Isaac Sim Warehouse Scene Creation
Creates a photorealistic warehouse environment for humanoid robot simulation.
Requires: Isaac Sim 2023.1.0+ with Omniverse Kit SDK
"""

import omni
from omni.isaac.kit import SimulationApp

# Launch Isaac Sim headless or with GUI
simulation_app = SimulationApp({"headless": False})

import carb
import numpy as np
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics, PhysxSchema
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.utils.prims import create_prim, define_prim
from omni.isaac.core.materials import PhysicsMaterial, PreviewSurface


def create_warehouse_floor(stage, world):
    """Create warehouse floor with realistic PBR concrete material."""
    print("[1/6] Creating warehouse floor...")

    # Define floor geometry (30m x 40m)
    floor_path = "/World/Floor"
    floor = create_prim(
        prim_path=floor_path,
        prim_type="Xform",
    )

    # Add mesh geometry
    mesh_path = f"{floor_path}/mesh"
    mesh = UsdGeom.Mesh.Define(stage, mesh_path)

    # Define plane vertices (rectangle)
    half_width = 20.0  # 40m width
    half_length = 15.0  # 30m length
    vertices = [
        (-half_width, -half_length, 0.0),
        (half_width, -half_length, 0.0),
        (half_width, half_length, 0.0),
        (-half_width, half_length, 0.0),
    ]
    mesh.GetPointsAttr().Set(vertices)

    # Define face indices (two triangles)
    face_vertex_counts = [3, 3]
    face_vertex_indices = [0, 1, 2, 0, 2, 3]
    mesh.GetFaceVertexCountsAttr().Set(face_vertex_counts)
    mesh.GetFaceVertexIndicesAttr().Set(face_vertex_indices)

    # UV coordinates for texture mapping
    uv_coords = [(0, 0), (10, 0), (10, 8), (0, 8)]  # Repeat texture 10x8 times
    primvar = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying
    )
    primvar.Set(uv_coords)

    # Apply PBR concrete material
    material = PreviewSurface(prim_path=f"{floor_path}/material")
    material.SetDiffuseTexture("omniverse://localhost/NVIDIA/Materials/Base/Flooring/Concrete_Polished.mdl")
    material.SetRoughness(0.7)
    material.SetMetallic(0.0)
    material.set_material(mesh_path)

    # Add physics collider
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    PhysxSchema.PhysxCollisionAPI.Apply(mesh.GetPrim())

    # Set friction properties (concrete on rubber)
    physics_material = PhysicsMaterial(
        prim_path=f"{floor_path}/physics_material",
        static_friction=1.0,
        dynamic_friction=0.8,
        restitution=0.1,
    )
    physics_material.apply_physics_material(mesh.GetPrim())

    print("  ✓ Floor created with PBR concrete material")


def create_shelving_units(stage, world):
    """Create warehouse shelving in 4 rows with 10 units each."""
    print("[2/6] Creating shelving units...")

    shelf_width = 2.0
    shelf_depth = 1.0
    shelf_height = 3.0
    rows = 4
    units_per_row = 10
    row_spacing = 6.0  # 6m between rows
    unit_spacing = 3.0  # 3m between units

    shelves_created = 0

    for row in range(rows):
        for unit in range(units_per_row):
            # Calculate position
            x = -18.0 + unit * unit_spacing
            y = -10.0 + row * row_spacing
            z = shelf_height / 2.0

            shelf_path = f"/World/Shelves/Row{row}/Unit{unit}"

            # Create shelf frame (simplified as 3 stacked boxes)
            for level in range(3):
                level_z = z + (level - 1) * 1.0
                level_path = f"{shelf_path}/Level{level}"

                # Shelf surface
                shelf_surface = create_prim(
                    prim_path=level_path,
                    prim_type="Cube",
                    position=np.array([x, y, level_z]),
                    scale=np.array([shelf_width, shelf_depth, 0.05]),
                )

                # Apply metal material
                UsdGeom.Gprim(stage.GetPrimAtPath(level_path)).CreateDisplayColorAttr([(0.6, 0.6, 0.7)])

                # Add collider
                UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(level_path))
                UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath(level_path))

                # Make static (shelves don't move)
                rigid_body = UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath(level_path))
                rigid_body.CreateKinematicEnabledAttr(True)

            shelves_created += 1

    print(f"  ✓ Created {shelves_created} shelving units in {rows} rows")


def create_pallet_stacks(stage, world):
    """Create dynamic pallet stacks for manipulation testing."""
    print("[3/6] Creating pallet stacks...")

    pallet_positions = [
        (5.0, -8.0, 0.2),
        (8.0, -8.0, 0.2),
        (5.0, 5.0, 0.2),
        (-10.0, 10.0, 0.2),
    ]

    for idx, pos in enumerate(pallet_positions):
        # Create pallet base
        pallet_path = f"/World/Pallets/Pallet{idx}"
        pallet = create_prim(
            prim_path=pallet_path,
            prim_type="Cube",
            position=np.array(pos),
            scale=np.array([1.0, 1.0, 0.15]),
        )

        # Apply wood material
        UsdGeom.Gprim(stage.GetPrimAtPath(pallet_path)).CreateDisplayColorAttr([(0.6, 0.4, 0.2)])

        # Add physics
        UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(pallet_path))
        UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath(pallet_path))
        mass_api = UsdPhysics.MassAPI.Apply(stage.GetPrimAtPath(pallet_path))
        mass_api.CreateMassAttr(50.0)  # 50 kg pallet

        # Stack 2-4 boxes on each pallet
        num_boxes = np.random.randint(2, 5)
        for box_idx in range(num_boxes):
            box_path = f"{pallet_path}/Box{box_idx}"
            box_z = pos[2] + 0.15 + (box_idx + 1) * 0.4

            box = create_prim(
                prim_path=box_path,
                prim_type="Cube",
                position=np.array([pos[0], pos[1], box_z]),
                scale=np.array([0.4, 0.4, 0.4]),
            )

            # Cardboard box color
            UsdGeom.Gprim(stage.GetPrimAtPath(box_path)).CreateDisplayColorAttr([(0.8, 0.7, 0.5)])

            # Add physics (lighter boxes)
            UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(box_path))
            UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath(box_path))
            mass_api = UsdPhysics.MassAPI.Apply(stage.GetPrimAtPath(box_path))
            mass_api.CreateMassAttr(10.0)  # 10 kg box

    print(f"  ✓ Created {len(pallet_positions)} pallet stacks with boxes")


def create_dome_lighting(stage):
    """Create HDR dome light for realistic warehouse illumination."""
    print("[4/6] Setting up dome lighting...")

    # Create dome light
    dome_light_path = "/World/Lights/DomeLight"
    dome_light = UsdLux.DomeLight.Define(stage, dome_light_path)

    # Load HDR environment texture (warehouse interior)
    dome_light.CreateTextureFileAttr("omniverse://localhost/NVIDIA/Assets/Skies/Indoor/warehouse_01.hdr")
    dome_light.CreateIntensityAttr(1000.0)
    dome_light.CreateTextureFormatAttr(UsdLux.Tokens.latlong)

    # Add fill lights for shadows
    fill_light_path = "/World/Lights/FillLight"
    fill_light = UsdLux.RectLight.Define(stage, fill_light_path)
    fill_light.CreateIntensityAttr(5000.0)
    fill_light.CreateWidthAttr(10.0)
    fill_light.CreateHeightAttr(10.0)
    UsdGeom.Xformable(fill_light).AddTranslateOp().Set(Gf.Vec3d(0, 0, 8.0))
    UsdGeom.Xformable(fill_light).AddRotateXYZOp().Set(Gf.Vec3d(-90, 0, 0))

    print("  ✓ HDR dome light and fill light configured")


def spawn_humanoid_robot(stage, world):
    """Spawn humanoid robot at designated start position."""
    print("[5/6] Spawning humanoid robot...")

    # Robot spawn position (center of warehouse, clear area)
    robot_path = "/World/Robots/Humanoid"
    robot_pos = np.array([0.0, 0.0, 1.05])

    # Load robot from USD (assumes pre-built humanoid asset)
    # Replace with your actual robot USD path
    robot_usd_path = "omniverse://localhost/Projects/Humanoids/humanoid_v1.usd"

    robot = create_prim(
        prim_path=robot_path,
        prim_type="Xform",
        position=robot_pos,
        usd_path=robot_usd_path,
    )

    # Add camera to robot head
    camera_path = f"{robot_path}/Head/Camera"
    camera = create_prim(prim_path=camera_path, prim_type="Camera")
    camera_prim = stage.GetPrimAtPath(camera_path)

    # Configure camera parameters
    camera_api = UsdGeom.Camera(camera_prim)
    camera_api.CreateFocalLengthAttr(24.0)  # Wide-angle lens
    camera_api.CreateFocusDistanceAttr(5.0)
    camera_api.CreateFStopAttr(2.8)  # Shallow depth of field
    camera_api.CreateHorizontalApertureAttr(20.955)
    camera_api.CreateVerticalApertureAttr(15.2908)

    # Add camera transform (point forward)
    xform = UsdGeom.Xformable(camera_prim)
    xform.AddTranslateOp().Set(Gf.Vec3d(0.1, 0, 0.6))
    xform.AddRotateXYZOp().Set(Gf.Vec3d(0, 0, 0))

    print(f"  ✓ Robot spawned at {robot_pos}")


def configure_physics_scene(world):
    """Configure global physics parameters for warehouse simulation."""
    print("[6/6] Configuring physics scene...")

    # Set physics timestep (small for stable humanoid simulation)
    world.get_physics_context().set_physics_dt(1.0 / 500.0)  # 500 Hz

    # Enable GPU dynamics (PhysX GPU acceleration)
    world.get_physics_context().enable_gpu_dynamics(True)

    # Set gravity
    world.get_physics_context().set_gravity(-9.81)

    # Solver iterations (higher = more stable contacts)
    world.get_physics_context().set_solver_type("TGS")  # Temporal Gauss-Seidel

    print("  ✓ Physics configured: 500 Hz, GPU dynamics enabled")


def main():
    """Main scene creation workflow."""
    print("\n=== Isaac Sim Warehouse Scene Creation ===\n")

    # Initialize world
    world = World(stage_units_in_meters=1.0)
    stage = omni.usd.get_context().get_stage()

    # Create scene elements
    create_warehouse_floor(stage, world)
    create_shelving_units(stage, world)
    create_pallet_stacks(stage, world)
    create_dome_lighting(stage)
    spawn_humanoid_robot(stage, world)
    configure_physics_scene(world)

    # Save scene
    output_path = "omniverse://localhost/Projects/Warehouses/warehouse_scene.usd"
    stage.Export(output_path)
    print(f"\n✓ Scene saved to: {output_path}")

    # Run simulation for 5 seconds to verify physics
    print("\nRunning simulation test (5 seconds)...")
    world.reset()

    for i in range(2500):  # 5 seconds at 500 Hz
        world.step(render=True)
        if i % 500 == 0:
            print(f"  Simulation step: {i}/2500")

    print("\n=== Scene creation complete! ===\n")
    print("Next steps:")
    print("  1. Open scene in Isaac Sim: File → Open → warehouse_scene.usd")
    print("  2. Configure robot sensors (Chapter 4)")
    print("  3. Set up Replicator for synthetic data (Example 2)")

    simulation_app.close()


if __name__ == "__main__":
    main()
```

### Key Annotations

1. **PBR Materials**: Concrete floor uses MDL material with realistic roughness/metallic properties.
2. **HDR Dome Lighting**: Warehouse HDR texture provides realistic ambient occlusion and reflections.
3. **Physics Properties**: Static friction (μ = 1.0) for concrete, kinematic shelves (immovable).
4. **GPU Acceleration**: `enable_gpu_dynamics(True)` leverages PhysX GPU for 10–100× faster simulation.
5. **Camera Configuration**: Wide-angle lens (24 mm focal length) with realistic aperture settings.
6. **Timestep**: 500 Hz (0.002 s) ensures stable contact dynamics for bipedal locomotion.

### Expected Output

```
=== Isaac Sim Warehouse Scene Creation ===

[1/6] Creating warehouse floor...
  ✓ Floor created with PBR concrete material
[2/6] Creating shelving units...
  ✓ Created 40 shelving units in 4 rows
[3/6] Creating pallet stacks...
  ✓ Created 4 pallet stacks with boxes
[4/6] Setting up dome lighting...
  ✓ HDR dome light and fill light configured
[5/6] Spawning humanoid robot...
  ✓ Robot spawned at [0. 0. 1.05]
[6/6] Configuring physics scene...
  ✓ Physics configured: 500 Hz, GPU dynamics enabled

✓ Scene saved to: omniverse://localhost/Projects/Warehouses/warehouse_scene.usd

Running simulation test (5 seconds)...
  Simulation step: 0/2500
  Simulation step: 500/2500
  ...
```

### References

- Chapter 1: Isaac Sim Architecture & Python API
- Chapter 2: PBR Materials & Lighting
- Chapter 5: Physics Simulation Parameters

---

## Example 2: Replicator API - Synthetic Data Generation

This example demonstrates using the Isaac Sim Replicator API to generate 1000 randomized RGB images with bounding box annotations in COCO format for training perception models.

### Scenario

Generate synthetic training data for object detection:
- Randomize: Robot pose, camera viewpoint, lighting intensity, object positions
- Annotate: 2D bounding boxes, instance segmentation masks, depth maps
- Export: COCO JSON format for YOLOv8/Detectron2 training

### Python Script: `generate_synthetic_data.py`

```python
#!/usr/bin/env python3
"""
Isaac Sim Replicator - Synthetic Data Generation
Generates randomized RGB+depth images with COCO annotations for object detection.
Requires: Isaac Sim 2023.1.0+ with Replicator extension
"""

import omni.replicator.core as rep
from omni.isaac.kit import SimulationApp

# Launch Isaac Sim headless for batch data generation
simulation_app = SimulationApp({"headless": True})

import json
import numpy as np
from pathlib import Path
from datetime import datetime


def setup_scene_randomization():
    """Define scene randomization graph using Replicator API."""
    print("[1/4] Setting up scene randomization...")

    # Load warehouse scene (from Example 1)
    scene_path = "omniverse://localhost/Projects/Warehouses/warehouse_scene.usd"
    rep.create.from_usd(scene_path)

    # Define randomization targets
    robot = rep.get.prims(path_pattern="/World/Robots/Humanoid")
    pallets = rep.get.prims(path_pattern="/World/Pallets/*")
    boxes = rep.get.prims(path_pattern="/World/Pallets/*/Box*")
    camera = rep.get.prims(path_pattern="/World/Robots/Humanoid/Head/Camera")
    lights = rep.get.prims(path_pattern="/World/Lights/*")

    # Randomization functions
    with rep.trigger.on_frame(num_frames=1000):  # Generate 1000 frames

        # 1. Randomize robot pose (position + orientation)
        with robot:
            rep.modify.pose(
                position=rep.distribution.uniform((-15, -12, 1.05), (15, 12, 1.05)),
                rotation=rep.distribution.uniform((0, 0, -180), (0, 0, 180)),
            )

        # 2. Randomize object positions (pallets and boxes)
        with pallets:
            rep.modify.pose(
                position=rep.distribution.uniform((-18, -10, 0.2), (18, 10, 0.2)),
                rotation=rep.distribution.uniform((0, 0, -30), (0, 0, 30)),
            )

        with boxes:
            # Keep boxes on pallets but randomize stacking
            rep.modify.pose(
                position=rep.distribution.uniform((-0.2, -0.2, 0.3), (0.2, 0.2, 1.5)),
                rotation=rep.distribution.uniform((0, 0, -15), (0, 0, 15)),
            )

        # 3. Randomize lighting (intensity + color temperature)
        with lights:
            rep.modify.attribute(
                "intensity",
                rep.distribution.uniform(800, 1500)
            )
            rep.modify.attribute(
                "color",
                rep.distribution.uniform((0.9, 0.9, 0.95), (1.0, 0.98, 0.95))  # Cool to warm white
            )

        # 4. Randomize camera parameters (focal length, aperture)
        with camera:
            rep.modify.attribute(
                "focalLength",
                rep.distribution.choice([18, 24, 35, 50])  # Common lens focal lengths
            )
            rep.modify.attribute(
                "fStop",
                rep.distribution.uniform(2.8, 8.0)  # Aperture range
            )

    print("  ✓ Randomization graph configured")
    return camera


def configure_annotations(camera):
    """Configure Replicator writers for RGB, depth, and bounding boxes."""
    print("[2/4] Configuring annotation writers...")

    # Output directory
    output_dir = f"_out_synthetic_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    Path(output_dir).mkdir(exist_ok=True)

    # Render product: Attach annotators to camera
    render_product = rep.create.render_product(camera, resolution=(1280, 720))

    # RGB writer
    rgb_writer = rep.WriterRegistry.get("BasicWriter")
    rgb_writer.initialize(
        output_dir=f"{output_dir}/rgb",
        rgb=True,
        colorize_semantic_segmentation=False,
    )
    rgb_writer.attach([render_product])

    # Depth writer (normalized 16-bit PNG)
    depth_writer = rep.WriterRegistry.get("BasicWriter")
    depth_writer.initialize(
        output_dir=f"{output_dir}/depth",
        distance_to_camera=True,
        colorize_distance_to_camera=False,
    )
    depth_writer.attach([render_product])

    # Bounding box writer (COCO format)
    bbox_writer = rep.WriterRegistry.get("COCOWriter")
    bbox_writer.initialize(
        output_dir=f"{output_dir}/annotations",
        bbox_2d_tight=True,
        bbox_2d_loose=False,
        semantic_segmentation=True,
        instance_segmentation=True,
    )
    bbox_writer.attach([render_product])

    print(f"  ✓ Writers configured, output: {output_dir}")
    return output_dir


def define_semantic_classes():
    """Define semantic classes for annotation."""
    print("[3/4] Defining semantic classes...")

    # Map scene objects to semantic labels
    semantic_classes = [
        {"class": "pallet", "color": (0.8, 0.6, 0.4)},
        {"class": "box", "color": (0.9, 0.8, 0.6)},
        {"class": "shelf", "color": (0.7, 0.7, 0.8)},
        {"class": "floor", "color": (0.5, 0.5, 0.5)},
        {"class": "robot", "color": (0.3, 0.5, 0.8)},
    ]

    # Apply semantic labels to prims
    with rep.get.prims(path_pattern="/World/Pallets/Pallet*"):
        rep.modify.semantics([("class", "pallet")])

    with rep.get.prims(path_pattern="/World/Pallets/*/Box*"):
        rep.modify.semantics([("class", "box")])

    with rep.get.prims(path_pattern="/World/Shelves/*/*"):
        rep.modify.semantics([("class", "shelf")])

    with rep.get.prims(path_pattern="/World/Floor"):
        rep.modify.semantics([("class", "floor")])

    with rep.get.prims(path_pattern="/World/Robots/Humanoid"):
        rep.modify.semantics([("class", "robot")])

    print(f"  ✓ {len(semantic_classes)} semantic classes defined")
    return semantic_classes


def run_data_generation(num_frames=1000):
    """Execute Replicator orchestrator to generate data."""
    print(f"[4/4] Generating {num_frames} frames...")

    # Run orchestrator
    rep.orchestrator.run()

    # Progress tracking
    for i in range(num_frames):
        rep.orchestrator.step()

        if (i + 1) % 100 == 0:
            print(f"  Generated {i + 1}/{num_frames} frames ({(i+1)/num_frames*100:.1f}%)")

    rep.orchestrator.stop()
    print("  ✓ Data generation complete")


def verify_coco_annotations(output_dir):
    """Load and verify COCO annotations."""
    print("\n[Verification] Checking COCO annotations...")

    coco_json_path = Path(output_dir) / "annotations" / "coco_annotations.json"

    if not coco_json_path.exists():
        print("  ⚠ COCO JSON not found. Check writer configuration.")
        return

    with open(coco_json_path, 'r') as f:
        coco_data = json.load(f)

    num_images = len(coco_data.get("images", []))
    num_annotations = len(coco_data.get("annotations", []))
    num_categories = len(coco_data.get("categories", []))

    print(f"  ✓ Images: {num_images}")
    print(f"  ✓ Annotations: {num_annotations}")
    print(f"  ✓ Categories: {num_categories}")

    # Sample annotation
    if num_annotations > 0:
        sample_ann = coco_data["annotations"][0]
        print(f"\n  Sample annotation:")
        print(f"    Image ID: {sample_ann['image_id']}")
        print(f"    Category ID: {sample_ann['category_id']}")
        print(f"    Bounding Box (XYWH): {sample_ann['bbox']}")
        print(f"    Area: {sample_ann['area']:.2f} px²")


def main():
    """Main synthetic data generation workflow."""
    print("\n=== Isaac Sim Replicator - Synthetic Data Generation ===\n")

    # Setup pipeline
    camera = setup_scene_randomization()
    output_dir = configure_annotations(camera)
    semantic_classes = define_semantic_classes()

    # Generate data
    run_data_generation(num_frames=1000)

    # Verify output
    verify_coco_annotations(output_dir)

    print(f"\n=== Generation complete! ===")
    print(f"\nOutput structure:")
    print(f"  {output_dir}/")
    print(f"    rgb/              # 1000 RGB images (1280×720)")
    print(f"    depth/            # 1000 depth maps (16-bit PNG)")
    print(f"    annotations/      # COCO JSON + segmentation masks")
    print(f"\nNext steps:")
    print(f"  1. Train YOLOv8: yolov8 train --data {output_dir}/annotations/coco.yaml")
    print(f"  2. Visualize: python visualize_coco.py --json {output_dir}/annotations/coco_annotations.json")
    print(f"  3. Domain randomization tuning (Chapter 3)")

    simulation_app.close()


if __name__ == "__main__":
    main()
```

### Key Annotations

1. **Randomization Distributions**: `uniform()` for continuous ranges, `choice()` for discrete options.
2. **Trigger**: `on_frame(num_frames=1000)` generates exactly 1000 randomized frames.
3. **Semantic Labels**: `rep.modify.semantics()` assigns class labels for annotation.
4. **COCO Writer**: Automatically generates `coco_annotations.json` with `bbox`, `area`, `segmentation`.
5. **Multi-Modal Output**: RGB (8-bit), depth (16-bit), segmentation masks (32-bit instance IDs).

### Expected Output

```
=== Isaac Sim Replicator - Synthetic Data Generation ===

[1/4] Setting up scene randomization...
  ✓ Randomization graph configured
[2/4] Configuring annotation writers...
  ✓ Writers configured, output: _out_synthetic_data_20250610_143022
[3/4] Defining semantic classes...
  ✓ 5 semantic classes defined
[4/4] Generating 1000 frames...
  Generated 100/1000 frames (10.0%)
  Generated 200/1000 frames (20.0%)
  ...
  Generated 1000/1000 frames (100.0%)
  ✓ Data generation complete

[Verification] Checking COCO annotations...
  ✓ Images: 1000
  ✓ Annotations: 3847
  ✓ Categories: 5

  Sample annotation:
    Image ID: 0
    Category ID: 2
    Bounding Box (XYWH): [342.5, 198.3, 156.2, 203.8]
    Area: 31839.56 px²
```

### References

- Chapter 3: Replicator API & Domain Randomization
- Chapter 4: Semantic Segmentation for Annotations

---

## Example 3: Isaac ROS Visual-Inertial SLAM (vSLAM) Launch

This example demonstrates a complete ROS 2 launch file for Isaac ROS visual SLAM with stereo cameras and IMU fusion for robust localization.

### Scenario

Configure Isaac ROS vSLAM for a humanoid robot with:
- Stereo camera pair (left/right RGB)
- IMU for gravity alignment and drift correction
- Loop closure detection for long-term consistency
- Real-time pose output at 30 Hz

### Launch File: `isaac_vslam.launch.py`

```python
#!/usr/bin/env python3
"""
Isaac ROS Visual-Inertial SLAM Launch File
Configures stereo vSLAM with IMU fusion for humanoid robot localization.
Requires: Isaac ROS 2.0+, isaac_ros_visual_slam package
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for Isaac ROS vSLAM pipeline."""

    # Package directories
    pkg_isaac_vslam = get_package_share_directory('isaac_ros_visual_slam')
    pkg_robot_description = get_package_share_directory('humanoid_description')

    # Configuration file paths
    vslam_config = os.path.join(pkg_isaac_vslam, 'config', 'vslam_stereo_imu.yaml')
    rviz_config = os.path.join(pkg_isaac_vslam, 'rviz', 'vslam_visualization.rviz')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    enable_imu = LaunchConfiguration('enable_imu', default='true')
    enable_localization_n_mapping = LaunchConfiguration('enable_localization_n_mapping', default='true')
    publish_map_to_odom_tf = LaunchConfiguration('publish_map_to_odom_tf', default='true')

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time from Gazebo/Isaac Sim'
    )

    declare_enable_imu = DeclareLaunchArgument(
        'enable_imu',
        default_value='true',
        description='Enable IMU fusion for drift correction'
    )

    # ============================================================
    # 1. Image Rectification Nodes (isaac_ros_image_proc)
    # ============================================================

    rectify_container = ComposableNodeContainer(
        name='rectify_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',  # Multi-threaded for GPU acceleration
        composable_node_descriptions=[
            # Left camera rectification
            ComposableNode(
                package='isaac_ros_image_proc',
                plugin='nvidia::isaac_ros::image_proc::RectifyNode',
                name='rectify_left',
                parameters=[{
                    'use_sim_time': use_sim_time,
                }],
                remappings=[
                    ('image_raw', '/camera/left/image_raw'),
                    ('camera_info', '/camera/left/camera_info'),
                    ('image_rect', '/camera/left/image_rect'),
                    ('camera_info_rect', '/camera/left/camera_info_rect'),
                ]
            ),
            # Right camera rectification
            ComposableNode(
                package='isaac_ros_image_proc',
                plugin='nvidia::isaac_ros::image_proc::RectifyNode',
                name='rectify_right',
                parameters=[{
                    'use_sim_time': use_sim_time,
                }],
                remappings=[
                    ('image_raw', '/camera/right/image_raw'),
                    ('camera_info', '/camera/right/camera_info'),
                    ('image_rect', '/camera/right/image_rect'),
                    ('camera_info_rect', '/camera/right/camera_info_rect'),
                ]
            ),
        ],
        output='screen',
    )

    # ============================================================
    # 2. Visual SLAM Node
    # ============================================================

    vslam_node = ComposableNodeContainer(
        name='vslam_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_visual_slam',
                plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
                name='visual_slam',
                parameters=[{
                    'use_sim_time': use_sim_time,

                    # IMU configuration
                    'enable_imu_fusion': enable_imu,
                    'gyro_noise_density': 0.0002,       # rad/s/√Hz (typical MEMS IMU)
                    'gyro_random_walk': 0.00002,        # rad/s²/√Hz
                    'accel_noise_density': 0.017,       # m/s²/√Hz
                    'accel_random_walk': 0.002,         # m/s³/√Hz
                    'imu_calibration_mode': 'static',   # Static initialization

                    # Stereo camera configuration
                    'num_cameras': 2,
                    'min_num_images': 2,
                    'rectified_images': True,
                    'stereo_baseline': 0.12,            # 12 cm baseline (typical for humanoid)

                    # Feature tracking parameters
                    'enable_observations_view': True,
                    'enable_landmarks_view': True,
                    'enable_reading_slam_internals': True,
                    'enable_slam_visualization': True,
                    'enable_localization_n_mapping': enable_localization_n_mapping,

                    # Map management
                    'map_frame': 'map',
                    'odom_frame': 'odom',
                    'base_frame': 'base_link',
                    'publish_map_to_odom_tf': publish_map_to_odom_tf,

                    # Loop closure
                    'enable_loop_closure': True,
                    'loop_closure_frequency': 5.0,      # Check every 5 seconds
                    'loop_closure_radius': 5.0,         # 5 meter search radius

                    # Performance tuning
                    'gpu_id': 0,
                    'enable_verbosity': False,
                }],
                remappings=[
                    ('stereo_camera/left/image', '/camera/left/image_rect'),
                    ('stereo_camera/left/camera_info', '/camera/left/camera_info_rect'),
                    ('stereo_camera/right/image', '/camera/right/image_rect'),
                    ('stereo_camera/right/camera_info', '/camera/right/camera_info_rect'),
                    ('visual_slam/imu', '/imu/data'),
                    ('visual_slam/pose', '/vslam/pose'),
                    ('visual_slam/tracking/odometry', '/vslam/odometry'),
                    ('visual_slam/tracking/vo_pose', '/vslam/vo_pose'),
                    ('visual_slam/tracking/slam_path', '/vslam/path'),
                ]
            ),
        ],
        output='screen',
    )

    # ============================================================
    # 3. Transform Broadcaster (base_link → camera frames)
    # ============================================================

    static_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf_broadcaster',
        arguments=[
            '0.1', '0.05', '0.6',    # Translation (x, y, z) from base_link
            '0', '0', '0', '1',      # Rotation quaternion (no rotation)
            'base_link',
            'camera_left',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    static_tf_publisher_right = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_right_tf_broadcaster',
        arguments=[
            '0.1', '-0.07', '0.6',   # 12 cm to the right of left camera
            '0', '0', '0', '1',
            'base_link',
            'camera_right',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ============================================================
    # 4. RViz Visualization
    # ============================================================

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    # ============================================================
    # Launch Description
    # ============================================================

    return LaunchDescription([
        declare_use_sim_time,
        declare_enable_imu,

        # Image processing
        rectify_container,

        # Visual SLAM
        vslam_node,

        # Transforms
        static_tf_publisher,
        static_tf_publisher_right,

        # Visualization
        rviz_node,
    ])
```

### Key Annotations

1. **Composable Nodes**: All nodes run in a single process (`component_container_mt`) for zero-copy message passing via GPU.
2. **IMU Noise Parameters**: Realistic MEMS IMU specs (noise density, random walk) from datasheets.
3. **Stereo Baseline**: 12 cm separation between cameras (typical for humanoid head width).
4. **Loop Closure**: Checks every 5 seconds within 5 m radius for drift correction.
5. **TF Frames**: `map` → `odom` → `base_link` → `camera_left`/`camera_right`.

### Usage

```bash
# Launch vSLAM with default parameters
ros2 launch humanoid_perception isaac_vslam.launch.py

# Launch without IMU (visual-only SLAM)
ros2 launch humanoid_perception isaac_vslam.launch.py enable_imu:=false

# Localization mode (use existing map, no new landmarks)
ros2 launch humanoid_perception isaac_vslam.launch.py enable_localization_n_mapping:=false
```

### Expected Topics

```bash
ros2 topic list

# Outputs:
/camera/left/image_raw
/camera/left/camera_info
/camera/right/image_raw
/camera/right/camera_info
/imu/data
/vslam/pose                    # PoseStamped (30 Hz)
/vslam/odometry                # Odometry (30 Hz)
/vslam/path                    # Path (visualization)
/vslam/tracking/observations   # Feature tracks
/vslam/tracking/landmarks      # 3D map points
```

### Verification

1. **Check pose output**:
   ```bash
   ros2 topic echo /vslam/pose --field pose.position
   ```

2. **View feature tracks in RViz**:
   - Add `Image` display → topic `/vslam/tracking/observations`
   - Green dots = active features, red = lost tracks

3. **Check loop closures**:
   ```bash
   ros2 topic echo /vslam/tracking/loop_closure_count
   ```

### References

- Chapter 4: Isaac ROS Perception Pipeline
- Chapter 5: vSLAM & Nav2 Integration

---

## Example 4: TensorRT Model Conversion - YOLOv8 Optimization

This example shows end-to-end conversion of a PyTorch YOLOv8 model to TensorRT with FP16 precision for 10× faster inference on Jetson AGX Orin.

### Scenario

Optimize YOLOv8n (nano) for real-time object detection:
- Convert: PyTorch → ONNX → TensorRT
- Precision: FP16 (half-precision) for 2× speedup
- Target: 640×640 input, batch size 1 (real-time streaming)
- Deployment: Jetson AGX Orin (70 TOPS INT8)

### Python Script: `convert_yolov8_tensorrt.py`

```python
#!/usr/bin/env python3
"""
TensorRT Model Conversion - YOLOv8 Optimization
Converts PyTorch YOLOv8 → ONNX → TensorRT with FP16 for real-time inference.
Requires: TensorRT 8.5+, CUDA 11.8+, onnx, ultralytics
"""

import torch
import tensorrt as trt
import onnx
import numpy as np
from pathlib import Path
from ultralytics import YOLO
import time


class TensorRTConverter:
    """Handles PyTorch → ONNX → TensorRT conversion pipeline."""

    def __init__(self, model_path, output_dir="models_optimized"):
        self.model_path = model_path
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(exist_ok=True)

        # TensorRT logger
        self.logger = trt.Logger(trt.Logger.INFO)

        print(f"Initialized TensorRT Converter")
        print(f"  Input model: {model_path}")
        print(f"  Output directory: {self.output_dir}")

    def export_to_onnx(self, input_shape=(1, 3, 640, 640)):
        """Step 1: Export PyTorch model to ONNX format."""
        print("\n[Step 1/3] Exporting PyTorch → ONNX...")

        # Load YOLOv8 model
        model = YOLO(self.model_path)

        # Export to ONNX
        onnx_path = self.output_dir / "yolov8n.onnx"
        model.export(
            format="onnx",
            imgsz=640,
            dynamic=False,  # Static shape for TensorRT optimization
            simplify=True,  # Simplify ONNX graph
            opset=12,       # ONNX opset version
        )

        # Move exported ONNX to output directory
        default_onnx_path = Path(self.model_path).with_suffix('.onnx')
        default_onnx_path.rename(onnx_path)

        # Verify ONNX model
        onnx_model = onnx.load(str(onnx_path))
        onnx.checker.check_model(onnx_model)

        print(f"  ✓ ONNX export complete: {onnx_path}")
        print(f"  ✓ Model validated successfully")

        return onnx_path

    def build_tensorrt_engine(self, onnx_path, precision="fp16"):
        """Step 2: Build TensorRT engine from ONNX with specified precision."""
        print(f"\n[Step 2/3] Building TensorRT engine ({precision.upper()})...")

        # Create builder and network
        builder = trt.Builder(self.logger)
        network = builder.create_network(
            1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH)
        )
        parser = trt.OnnxParser(network, self.logger)

        # Parse ONNX model
        print("  Parsing ONNX model...")
        with open(onnx_path, 'rb') as f:
            if not parser.parse(f.read()):
                print("  ✗ Failed to parse ONNX model:")
                for error in range(parser.num_errors):
                    print(f"    {parser.get_error(error)}")
                return None

        # Configure builder
        config = builder.create_builder_config()

        # Set memory pool size (8 GB for workspace)
        config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, 8 << 30)

        # Enable FP16 precision if requested
        if precision == "fp16":
            if builder.platform_has_fast_fp16:
                config.set_flag(trt.BuilderFlag.FP16)
                print("  ✓ FP16 precision enabled")
            else:
                print("  ⚠ FP16 not supported on this platform, using FP32")

        # Enable INT8 precision if requested (requires calibration data)
        if precision == "int8":
            if builder.platform_has_fast_int8:
                config.set_flag(trt.BuilderFlag.INT8)
                # TODO: Add INT8 calibrator here
                print("  ✓ INT8 precision enabled (calibration required)")
            else:
                print("  ⚠ INT8 not supported on this platform")

        # Build engine
        print("  Building optimized engine (may take 5-10 minutes)...")
        start_time = time.time()
        serialized_engine = builder.build_serialized_network(network, config)
        build_time = time.time() - start_time

        if serialized_engine is None:
            print("  ✗ Failed to build TensorRT engine")
            return None

        # Save engine to file
        engine_path = self.output_dir / f"yolov8n_{precision}.engine"
        with open(engine_path, 'wb') as f:
            f.write(serialized_engine)

        print(f"  ✓ Engine built in {build_time:.1f} seconds")
        print(f"  ✓ Saved to: {engine_path}")

        return engine_path

    def benchmark_inference(self, engine_path, num_iterations=100):
        """Step 3: Benchmark TensorRT engine performance."""
        print(f"\n[Step 3/3] Benchmarking inference ({num_iterations} iterations)...")

        # Load engine
        with open(engine_path, 'rb') as f:
            runtime = trt.Runtime(self.logger)
            engine = runtime.deserialize_cuda_engine(f.read())

        # Create execution context
        context = engine.create_execution_context()

        # Prepare input/output buffers
        import pycuda.driver as cuda
        import pycuda.autoinit  # Initializes CUDA

        # Get input/output shapes
        input_shape = engine.get_binding_shape(0)
        output_shape = engine.get_binding_shape(1)

        print(f"  Input shape: {input_shape}")
        print(f"  Output shape: {output_shape}")

        # Allocate device memory
        input_size = trt.volume(input_shape) * np.dtype(np.float32).itemsize
        output_size = trt.volume(output_shape) * np.dtype(np.float32).itemsize

        d_input = cuda.mem_alloc(input_size)
        d_output = cuda.mem_alloc(output_size)

        # Prepare dummy input (random noise)
        h_input = np.random.randn(*input_shape).astype(np.float32)
        h_output = np.empty(output_shape, dtype=np.float32)

        # Warmup (5 iterations)
        print("  Warming up...")
        for _ in range(5):
            cuda.memcpy_htod(d_input, h_input)
            context.execute_v2([int(d_input), int(d_output)])
            cuda.memcpy_dtoh(h_output, d_output)

        # Benchmark
        print(f"  Running {num_iterations} iterations...")
        timings = []

        for i in range(num_iterations):
            cuda.memcpy_htod(d_input, h_input)

            start = time.time()
            context.execute_v2([int(d_input), int(d_output)])
            cuda.Context.synchronize(pycuda.autoinit.context)
            end = time.time()

            timings.append((end - start) * 1000)  # Convert to ms

            if (i + 1) % 20 == 0:
                print(f"    Completed {i + 1}/{num_iterations} iterations")

        # Calculate statistics
        timings = np.array(timings)
        mean_time = np.mean(timings)
        std_time = np.std(timings)
        min_time = np.min(timings)
        max_time = np.max(timings)
        p95_time = np.percentile(timings, 95)
        fps = 1000.0 / mean_time

        print(f"\n  Performance Summary:")
        print(f"    Mean inference time: {mean_time:.2f} ms (± {std_time:.2f} ms)")
        print(f"    Min/Max: {min_time:.2f} / {max_time:.2f} ms")
        print(f"    P95 latency: {p95_time:.2f} ms")
        print(f"    Throughput: {fps:.1f} FPS")

        return {
            "mean": mean_time,
            "std": std_time,
            "min": min_time,
            "max": max_time,
            "p95": p95_time,
            "fps": fps,
        }


def compare_performance(pytorch_model_path, tensorrt_engine_path):
    """Compare PyTorch vs TensorRT performance."""
    print("\n[Performance Comparison] PyTorch vs TensorRT")

    # Benchmark PyTorch model
    print("\nBenchmarking PyTorch model...")
    model = YOLO(pytorch_model_path)
    dummy_input = torch.randn(1, 3, 640, 640).cuda()

    # Warmup
    for _ in range(5):
        model(dummy_input)

    # Benchmark
    pytorch_timings = []
    for _ in range(100):
        start = time.time()
        model(dummy_input)
        torch.cuda.synchronize()
        end = time.time()
        pytorch_timings.append((end - start) * 1000)

    pytorch_mean = np.mean(pytorch_timings)
    pytorch_fps = 1000.0 / pytorch_mean

    print(f"  PyTorch: {pytorch_mean:.2f} ms ({pytorch_fps:.1f} FPS)")

    # TensorRT results (from previous benchmark)
    print(f"\n  Speedup: {pytorch_mean / 8.5:.2f}× faster with TensorRT FP16")
    print(f"  (Assumes TensorRT achieved ~8.5 ms as shown in benchmark)")


def main():
    """Main TensorRT conversion workflow."""
    print("\n=== TensorRT YOLOv8 Optimization Pipeline ===\n")

    # Download YOLOv8n model (if not present)
    model_path = "yolov8n.pt"
    if not Path(model_path).exists():
        print("Downloading YOLOv8n model...")
        model = YOLO('yolov8n.pt')  # Auto-downloads from Ultralytics

    # Initialize converter
    converter = TensorRTConverter(model_path)

    # Conversion pipeline
    onnx_path = converter.export_to_onnx()
    engine_path = converter.build_tensorrt_engine(onnx_path, precision="fp16")

    if engine_path:
        benchmark_results = converter.benchmark_inference(engine_path)

        print("\n=== Conversion Complete! ===\n")
        print(f"Optimized model: {engine_path}")
        print(f"Expected speedup: 5-10× faster than PyTorch")
        print(f"\nDeployment example:")
        print(f"  1. Copy {engine_path.name} to robot")
        print(f"  2. Load with: runtime = trt.Runtime(logger)")
        print(f"  3. Run inference (see Chapter 4)")
    else:
        print("\n✗ Conversion failed. Check TensorRT installation.")


if __name__ == "__main__":
    main()
```

### Key Annotations

1. **ONNX Export**: `simplify=True` removes redundant nodes, `dynamic=False` enables TensorRT optimizations.
2. **FP16 Precision**: Halves memory usage, 2× speedup on Tensor Cores (Volta/Turing/Ampere GPUs).
3. **Workspace Memory**: 8 GB allows TensorRT to explore more optimization strategies.
4. **Benchmarking**: Includes warmup (5 iters) to stabilize GPU clocks before measurement.
5. **P95 Latency**: Critical for real-time systems (95th percentile response time).

### Expected Output

```
=== TensorRT YOLOv8 Optimization Pipeline ===

Initialized TensorRT Converter
  Input model: yolov8n.pt
  Output directory: models_optimized

[Step 1/3] Exporting PyTorch → ONNX...
  ✓ ONNX export complete: models_optimized/yolov8n.onnx
  ✓ Model validated successfully

[Step 2/3] Building TensorRT engine (FP16)...
  Parsing ONNX model...
  ✓ FP16 precision enabled
  Building optimized engine (may take 5-10 minutes)...
  ✓ Engine built in 347.2 seconds
  ✓ Saved to: models_optimized/yolov8n_fp16.engine

[Step 3/3] Benchmarking inference (100 iterations)...
  Input shape: (1, 3, 640, 640)
  Output shape: (1, 25200, 85)
  Warming up...
  Running 100 iterations...
    Completed 20/100 iterations
    ...
    Completed 100/100 iterations

  Performance Summary:
    Mean inference time: 8.47 ms (± 0.23 ms)
    Min/Max: 8.21 / 9.14 ms
    P95 latency: 8.89 ms
    Throughput: 118.1 FPS

=== Conversion Complete! ===
```

### References

- Chapter 4: TensorRT Optimization Techniques
- Chapter 5: Real-Time Object Detection Pipeline

---

## Example 5: Nav2 Custom Footstep Planner Configuration

This example demonstrates configuring Nav2 with a custom footstep planner for humanoid navigation, including YAML parameters and a Python behavior tree plugin.

### Scenario

Configure Nav2 for bipedal humanoid navigation:
- Custom footstep planner (replaces standard DWB controller)
- Costmap configuration for narrow spaces
- Behavior tree with recovery behaviors (re-planning, step adjustment)
- Integration with Isaac ROS vSLAM localization

### Configuration File: `nav2_humanoid_config.yaml`

```yaml
# Nav2 Configuration for Humanoid Footstep Navigation
# Replaces standard DWB local planner with footstep-aware controller

bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /vslam/odometry
    bt_loop_duration: 10
    default_server_timeout: 20

    # Behavior tree XML path
    default_nav_to_pose_bt_xml: "$(find-pkg-share humanoid_navigation)/behavior_trees/navigate_humanoid.xml"

    plugin_lib_names:
      - nav2_compute_path_to_pose_action_bt_node
      - nav2_follow_path_action_bt_node
      - nav2_back_up_action_bt_node
      - nav2_wait_action_bt_node
      - nav2_clear_costmap_service_bt_node
      - nav2_is_stuck_condition_bt_node
      - humanoid_footstep_planner_bt_node  # Custom plugin

bt_navigator_rclcpp_node:
  ros__parameters:
    use_sim_time: True


controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0  # 20 Hz control loop (matches footstep generation rate)

    # Custom footstep controller
    controller_plugins: ["FootstepController"]

    FootstepController:
      plugin: "humanoid_navigation::FootstepController"

      # Footstep parameters
      step_length: 0.25           # Maximum forward step length (m)
      step_width: 0.18            # Lateral foot separation (m)
      step_height: 0.05           # Foot clearance during swing phase (m)
      step_time: 0.6              # Time per step (s)

      # Kinematic constraints
      max_step_angle: 15.0        # Maximum turning angle per step (degrees)
      min_step_length: 0.10       # Minimum viable step (m)
      max_step_deviation: 0.05    # Maximum lateral deviation from plan (m)

      # Safety margins
      min_obstacle_distance: 0.3  # Stop if obstacle within 30 cm
      max_slope: 15.0             # Maximum climbable slope (degrees)

      # Path following
      lookahead_distance: 1.0     # How far ahead to plan footsteps (m)
      goal_tolerance_xy: 0.1      # XY position tolerance (m)
      goal_tolerance_theta: 0.2   # Angular tolerance (rad)


local_costmap:
  local_costmap:
    ros__parameters:
      use_sim_time: True
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true
      width: 5
      height: 5
      resolution: 0.05  # 5 cm resolution (fine for foot placement)
      update_frequency: 10.0
      publish_frequency: 5.0

      # Humanoid-specific footprint (rectangular, representing stance)
      footprint: "[[-0.15, -0.09], [-0.15, 0.09], [0.15, 0.09], [0.15, -0.09]]"

      plugins: ["obstacle_layer", "inflation_layer"]

      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan camera_depth

        # 2D LiDAR
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          min_obstacle_height: 0.1  # Ignore ground plane
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 10.0
          raytrace_min_range: 0.0
          obstacle_max_range: 8.0
          obstacle_min_range: 0.0

        # Depth camera (Isaac ROS)
        camera_depth:
          topic: /camera/depth/points
          max_obstacle_height: 2.0
          min_obstacle_height: 0.2
          clearing: True
          marking: True
          data_type: "PointCloud2"
          raytrace_max_range: 5.0
          obstacle_max_range: 4.0

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        enabled: True
        inflation_radius: 0.4  # 40 cm inflation (wider than humanoid body)
        cost_scaling_factor: 3.0


global_costmap:
  global_costmap:
    ros__parameters:
      use_sim_time: True
      global_frame: map
      robot_base_frame: base_link
      rolling_window: false  # Static map
      width: 50
      height: 50
      resolution: 0.1  # 10 cm resolution
      update_frequency: 1.0
      publish_frequency: 1.0

      footprint: "[[-0.15, -0.09], [-0.15, 0.09], [0.15, 0.09], [0.15, -0.09]]"

      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True

      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          min_obstacle_height: 0.1
          clearing: True
          marking: True
          data_type: "LaserScan"

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        enabled: True
        inflation_radius: 0.5  # Larger for global planning
        cost_scaling_factor: 5.0


planner_server:
  ros__parameters:
    use_sim_time: True
    planner_plugins: ["GridBased"]

    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: true  # A* for optimal paths
      allow_unknown: true


recovery_server:
  ros__parameters:
    use_sim_time: True
    recovery_plugins: ["backup", "wait", "clear_costmap"]

    backup:
      plugin: "nav2_recoveries/BackUp"
      backup_dist: 0.3      # Take 1-2 steps backward
      backup_speed: 0.1
      time_allowance: 10.0

    wait:
      plugin: "nav2_recoveries/Wait"
      wait_duration: 5.0

    clear_costmap:
      plugin: "nav2_recoveries/ClearCostmap"
      reset_distance: 3.0
```

### Key Annotations

1. **Footstep Controller**: Custom plugin replaces DWB; plans discrete foot placements instead of velocity commands.
2. **Fine Costmap Resolution**: 5 cm local costmap for precise foot placement planning.
3. **Humanoid Footprint**: Rectangular (30 cm × 18 cm) representing two-foot stance.
4. **Inflation Radius**: 40 cm ensures robot doesn't get too close to obstacles (body clearance).
5. **Recovery Behaviors**: Backup (step backward), wait (for dynamic obstacles), clear costmap (reset after sensor glitches).

### Python Footstep Controller Plugin (Simplified)

```python
# humanoid_navigation/footstep_controller.py
from nav2_core import Controller
import rclpy
from geometry_msgs.msg import Twist, PoseStamped
import numpy as np


class FootstepController(Controller):
    """Custom Nav2 controller that generates footstep plans for humanoid robots."""

    def configure(self, node, name, tf_buffer, costmap_ros):
        """Initialize controller parameters."""
        self.node = node
        self.tf_buffer = tf_buffer
        self.costmap = costmap_ros

        # Declare parameters
        node.declare_parameter(f"{name}.step_length", 0.25)
        node.declare_parameter(f"{name}.step_width", 0.18)
        node.declare_parameter(f"{name}.step_time", 0.6)

        # Get parameters
        self.step_length = node.get_parameter(f"{name}.step_length").value
        self.step_width = node.get_parameter(f"{name}.step_width").value
        self.step_time = node.get_parameter(f"{name}.step_time").value

        self.node.get_logger().info(f"FootstepController configured: step_length={self.step_length}m")

    def computeVelocityCommands(self, pose, velocity, goal):
        """
        Compute footstep-aware velocity commands.

        In a full implementation, this would:
        1. Discretize path into footstep locations
        2. Check footstep validity in costmap
        3. Generate swing trajectory for current step
        4. Output joint-space commands (not Twist)
        """
        # Simplified: Convert to linear/angular velocity
        # Real implementation would publish foot placement goals

        dx = goal.pose.position.x - pose.pose.position.x
        dy = goal.pose.position.y - pose.pose.position.y
        distance = np.hypot(dx, dy)

        cmd_vel = Twist()

        if distance > 0.1:
            cmd_vel.linear.x = min(self.step_length / self.step_time, 0.4)  # Max 0.4 m/s
            cmd_vel.angular.z = np.arctan2(dy, dx) * 0.5  # Gradual turning

        return cmd_vel
```

### Usage

```bash
# Launch Nav2 with humanoid configuration
ros2 launch humanoid_navigation navigation.launch.py params_file:=nav2_humanoid_config.yaml

# Send navigation goal
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "
pose:
  header:
    frame_id: 'map'
  pose:
    position: {x: 5.0, y: 3.0, z: 0.0}
    orientation: {w: 1.0}
"
```

### Expected Behavior

1. **Path Planning**: A* planner generates coarse path in global costmap.
2. **Footstep Generation**: Local controller discretizes path into viable foot placements.
3. **Obstacle Avoidance**: Footsteps avoid inflated obstacles (40 cm clearance).
4. **Recovery**: If stuck, robot steps backward or waits for dynamic obstacles to clear.

### References

- Chapter 5: vSLAM & Nav2 Integration
- Chapter 6: Humanoid Locomotion Control

---

## Summary

These examples demonstrate:

1. **Isaac Sim Scene Setup**: Programmatic creation of warehouse environment with PBR materials, HDR lighting, and physics configuration (200+ lines).
2. **Replicator API**: Automated synthetic data generation with domain randomization and COCO annotations (250+ lines).
3. **Isaac ROS vSLAM**: Complete launch file for stereo visual-inertial SLAM with IMU fusion (200+ lines).
4. **TensorRT Optimization**: End-to-end PyTorch → ONNX → TensorRT conversion with FP16 for 10× speedup (250+ lines).
5. **Nav2 Humanoid Configuration**: Custom footstep planner with humanoid-specific costmap and recovery behaviors (150+ YAML lines + Python plugin).

Use these examples as starting points for your GPU-accelerated perception and navigation projects. Modify scene complexity, randomization parameters, and controller settings to match your specific humanoid robot hardware and task requirements.

---

**Navigation**
← [Chapter 5: vSLAM & Nav2 Integration](/ai-robot-brain/vslam-nav2-integration)
→ [Exercises](/ai-robot-brain/exercises)
