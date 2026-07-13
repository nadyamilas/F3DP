"""
Experiments and generating the design options for the experiments.

Usage in grasshopper:
    from experiments import experiment_01
"""
from compas.geometry import Point, Vector, Polyline
from compas.datastructures import Mesh, Graph
import compas.topology as topology
import Rhino.Geometry as rg # type: ignore
import rhinoscriptsyntax as rs # type: ignore
import math 


# ------------------------------------------------------------------------
# Helpers
# ------------------------------------------------------------------------

def average_point(points):
    """Calculate the average point from a list of points."""
    x = sum(p.x for p in points) / len(points)
    y = sum(p.y for p in points) / len(points)
    z = sum(p.z for p in points) / len(points)
    return Point(x, y, z)

# --------------------------------------------------
# Initilaze primal mesh
# --------------------------------------------------

def create_mesh():
    primal_mesh = Mesh.from_meshgrid(dx=10, nx=5, dy=10, ny=5)

    # Ensure the primal mesh is fully triangulated (crucial for Kagome/Space frames)
    primal_mesh.quads_to_triangles()
    edges = primal_mesh.edges(data=True)

    return primal_mesh, edges


def exp_01(radius, num):
    """Experiment with spetial printing"""
    points = []
    all_angles = []
    alpha = (2 * math.pi) / num
    for n in range(num):
        x = math.sin(alpha * n) * radius
        y = math.cos(alpha * n) * radius
        z = 0 
        pt = Point(x, y, z)
        points.append(pt)
        all_angles.append(alpha)
    
    center = average_point(points)
    center_new = Point(center.x, center.y, center.z + 100)
    
    points.extend([points[1], center_new, points[-1], points[0], center_new, points[-2]])
    poly = Polyline(points)

    return poly, center_new, points

def exp_02():
    """Build basic mesh grid"""
    mesh = Mesh()
    primal_mesh = mesh.from_meshgrid(dx=100, nx=1, dy=100, ny=1)
    
    return primal_mesh, mesh

def exp_03(radius, num):
    """Build basic mesh grid"""
    points = []
    all_angles = []
    alpha = (2 * math.pi) / num
    for n in range(num):
        x = math.sin(alpha * n) * radius
        y = math.cos(alpha * n) * radius
        z = 0 
        pt = Point(x, y, z)
        points.append(pt)
        all_angles.append(alpha)
    
    center = average_point(points)
    center_new = Point(center.x, center.y, center.z + 100)
    points.append(center_new)
    mesh = Mesh.from_points(points)
        
    # center = average_point(points)
    # mesh.add_vertex(x=center.x, y=center.y, z=center.z + 100)
    
    mesh.quads_to_triangles(check_angles=False)
    
    return mesh

def create_trig_curve(type="sine", wavelength=10.0, amplitude=2.0, num_cycles=2, samples_per_cycle=20):
    """
    Generates a Sine or Cosine curve in Rhino and extracts manufacturing planes.
    
    Parameters:
    - type: "sine" or "cosine"
    - wavelength: The length of one complete wave cycle along the X-axis
    - amplitude: The peak height of the wave along the Y-axis
    - num_cycles: How many full waves to draw
    - samples_per_cycle: Controls the smoothness of the curve
    
    Returns:
    - tuple: (curve_id, planes_list)
    """
    points = []
    
    # Calculate total points to plot
    total_samples = int(num_cycles * samples_per_cycle)
    total_length = wavelength * num_cycles
    
    for i in range(total_samples + 1):
        # Calculate X coordinate
        x = (i / float(total_samples)) * total_length + 50
        
        # Calculate the angle in radians based on the current X position
        angle = (2 * math.pi * x) / wavelength
        
        # Calculate Y coordinate based on type
        if type.lower() == "cosine":
            y = amplitude * math.cos(angle) + i*8
        else:
            y = amplitude * math.sin(angle) + i*8
            
        # Z is kept at 0 for a flat 2D curve on the XY plane
        z = 0
        
        # Note: Keeps your original mapping orientation coordinate arrangement
        points.append(rg.Point3d(z, y, x))
    
    exit_point_start = rg.Point3d(points[-1].X, points[-1].Y, points[-1].Z + 100)
    exit_point = rg.Point3d(exit_point_start.X, exit_point_start.Y + 200, points[-1].Z + 200)
    points.append(exit_point_start)
    points.append(exit_point)
    
    
    # 1. Create and add the interpolated curve to the Rhino Document
    curve_geom = rg.Curve.CreateInterpolatedCurve(points, 3)
    curve_id = rs.AddInterpCurve(points)
    
    # # 2. Extract Perpendicular Planes for the robot extruder
    planes_out = []
    
    if curve_geom:
            t_params = curve_geom.DivideByCount(total_samples, True)
            for t_val in t_params:
                rc, perp_plane = curve_geom.PerpendicularFrameAt(t_val)
                if rc:
                    # --- CRITICAL ROBOT ALIGNMENT FIX ---
                    # Keep the exact location on the curve (Origin) 
                    # Keep the exact direction of travel (Normal/ZAxis)
                    origin = perp_plane.Origin
                    z_axis = perp_plane.ZAxis  # This is the path direction
                    
                    # Choose World X as our primary guide vector
                    world_guide = rg.Vector3d(1, 0, 0)
                    
                    # If the curve travels straight along World X, switch guide to World Y 
                    # to prevent mathematical breakdown (cross product with parallel vectors = 0)
                    if abs(z_axis.X) > 0.98:
                        world_guide = rg.Vector3d(0, 1, 0)
                    
                    # Compute a clean local Y-axis perpendicular to both the path and our world guide
                    local_y = rg.Vector3d.CrossProduct(z_axis, world_guide)
                    local_y.Unitize()
                    
                    # Compute the final corrected local X-axis to complete the perpendicular system
                    local_x = rg.Vector3d.CrossProduct(local_y, z_axis)
                    local_x.Unitize()
                    
                    # Construct the newly locked target plane
                    aligned_plane = rg.Plane(origin, local_x, local_y)
                    
                    planes_out.append(aligned_plane)
    
    entry_point = rg.Point3d(0, 300, 10)
    entry_plane = rg.Plane(entry_point, rg.Vector3d(1, 0, 0), rg.Vector3d(0, 1, 0))
    entry_point_start = rg.Point3d(0, points[0].Y, points[0].Z - 50)
    entry_plane_start = rg.Plane(entry_point_start, rg.Vector3d(1, 0, 0), rg.Vector3d(0, 1, 0))
    
    # exit_point_start = rg.Point3d(points[-1].X, points[-1].Y, points[-1].Z + 50)
    # exit_plane_end = rg.Plane(exit_point_start, rg.Vector3d(1, 0, 0), rg.Vector3d(0, 1, 0))
    # exit_point = rg.Point3d(300, exit_point_start.Y, exit_point_start.Z)
    # exit_plane = rg.Plane(exit_point, planes_out[-1].ZAxis, planes_out[-1].XAxis)
    
    # exit plane is the last plane in te list but moved for 300 in y directin

    
    planes_out.insert(0, entry_plane)  # Add the entry plane at the start of the list  
    planes_out.insert(1, entry_plane_start)  # Add the entry plane at the start of the list 
    # planes_out.append(exit_plane_end)
    # planes_out.append(exit_plane)  # Add the exit plane at the end of the list  
    
    return curve_id, planes_out


def create_trig_spiral_with_planes(radius=100.0, pitch=50.0, num_turns=4.0, amp=5.0, freq=12.0, planes_num=20):
    """
    Generates a 3D Sine-modulated spiral curve and extracts accurate printing planes.
    """
    points = []
    samples = int(num_turns * planes_num)
    
    for i in range(samples + 1):
        t = (i / float(samples)) * (num_turns * 2 * math.pi)
        
        current_radius = radius + amp * math.sin(t * freq)
        
        x = current_radius * math.cos(t)
        y = current_radius * math.sin(t)
        z = (t / (2 * math.pi)) * pitch  + 100
        
        points.append(rg.Point3d(x, y, z))
    
    exit_point_start = rg.Point3d(points[-1].X, points[-1].Y, points[-1].Z + 10)
    exit_point = rg.Point3d(exit_point_start.X, exit_point_start.Y + 200, points[-1].Z + 200)
    points.append(exit_point_start)
    points.append(exit_point)
    # 1. Create the interpolated 3D curve geometry
    curve_geom = rg.Curve.CreateInterpolatedCurve(points, 3)
    
    
    # Safely add the curve to the document and get its reference ID
    if 'scriptcontext' in globals():
        import scriptcontext
        curve_id = scriptcontext.doc.Objects.AddCurve(curve_geom)
    else:
        curve_id = rs.AddInterpCurve(points)
    
    # 2. Extract Perpendicular Planes for the robot tool vector
    planes_out = []
    
    if curve_geom:
            t_params = curve_geom.DivideByCount(samples, True)
            for t_val in t_params:
                rc, perp_plane = curve_geom.PerpendicularFrameAt(t_val)
                if rc:
                    # --- CRITICAL ROBOT ALIGNMENT FIX ---
                    # Keep the exact location on the curve (Origin) 
                    # Keep the exact direction of travel (Normal/ZAxis)
                    origin = perp_plane.Origin
                    z_axis = perp_plane.ZAxis  # This is the path direction
                    
                    # Choose World X as our primary guide vector
                    world_guide = rg.Vector3d(1, 0, 0)
                    
                    # If the curve travels straight along World X, switch guide to World Y 
                    # to prevent mathematical breakdown (cross product with parallel vectors = 0)
                    if abs(z_axis.X) > 0.98:
                        world_guide = rg.Vector3d(0, 1, 0)
                    
                    # Compute a clean local Y-axis perpendicular to both the path and our world guide
                    local_y = rg.Vector3d.CrossProduct(z_axis, world_guide)
                    local_y.Unitize()
                    
                    # Compute the final corrected local X-axis to complete the perpendicular system
                    local_x = rg.Vector3d.CrossProduct(local_y, z_axis)
                    local_x.Unitize()
                    
                    # Construct the newly locked target plane
                    aligned_plane = rg.Plane(origin, local_x, local_y)
                    
                    planes_out.append(aligned_plane)
    
    entry_point = rg.Point3d(0, 300, 10)
    entry_plane = rg.Plane(entry_point, rg.Vector3d(1, 0, 0), rg.Vector3d(0, 1, 0))
    entry_point_start = rg.Point3d(0, points[0].Y, points[0].Z - 100)
    entry_plane_start = rg.Plane(entry_point_start, rg.Vector3d(1, 0, 0), rg.Vector3d(0, 1, 0))
    
    # exit_point_start = rg.Point3d(0, points[-1].Y, points[-1].Z + 50)
    # exit_plane_end = rg.Plane(exit_point_start, rg.Vector3d(1, 0, 0), rg.Vector3d(0, 1, 0))
    # exit_point = rg.Point3d(300, exit_point_start.Y, exit_point_start.Z)
    # exit_plane = rg.Plane(exit_point, rg.Vector3d(1, 0, 0), rg.Vector3d(0, 1, 0))

    
    planes_out.insert(0, entry_plane)  # Add the entry plane at the start of the list  
    planes_out.insert(1, entry_plane_start)  # Add the entry plane at the start of the list 
    # planes_out.append(exit_plane_end)
    # planes_out.append(exit_plane)  # Add the exit plane at the end of the list  
                
    return curve_id, planes_out


# def create_vertical_spatial_bulb(target_height=400.0, amp=40.0, freq=4.0, samples=60, num=3, radius=100, spiral_turns=0.5):
#     """
#     Generates mid-air vertical printing curves that swell outwards horizontally,
#     spiral around a central axis, and converge at a top apex tip.
    
#     Parameters:
#     - target_height: The total vertical height (Z) where the apex point is located.
#     - amp: Amplitude of the sine wave 'wiggle' along the strands.
#     - freq: How many sine waves occur over the length of the strand.
#     - num: Number of starting strands.
#     - radius: Starting base radius from the center axis.
#     - spiral_turns: How many times each curve loops around the structure (e.g., 0.5 = half a turn).
#     """
#     start_points = []
#     alpha = (2 * math.pi) / num
    
#     # Generate symmetrical base anchor points automatically
#     for n in range(num):
#         x = math.sin(alpha * n) * radius
#         y = math.cos(alpha * n) * radius
#         z = 0 
#         start_points.append(rg.Point3d(x, y, z))
        
#     center_x = sum(p.X for p in start_points) / float(len(start_points))
#     center_y = sum(p.Y for p in start_points) / float(len(start_points))
#     center_z = sum(p.Z for p in start_points) / float(len(start_points))
    
#     base_center = rg.Point3d(center_x, center_y, center_z)
#     apex_point = rg.Point3d(base_center.X, base_center.Y, base_center.Z + target_height)
    
#     curves_out = []
#     planes_out = []
    
#     for p in start_points:
#         dx = p.X - base_center.X
#         dy = p.Y - base_center.Y
        
#         start_radius = math.sqrt(dx**2 + dy**2)
#         base_angle = math.atan2(dy, dx)
        
#         strand_points = []
        
#         for i in range(samples + 1):
#             t = i / float(samples)
            
#             # Linear transition upward
#             z = base_center.Z + (t * target_height)
            
#             # --- The Spiral Logic ---
#             # As t increases, the angle steadily rotates around the center axis
#             total_spiral_angle = spiral_turns * 2 * math.pi
#             current_angle = base_angle + (t * total_spiral_angle)
            
#             # --- The Vertical Bulb Shape Envelope ---
#             profile_envelope = math.sin(t * math.pi)
#             current_radius = (start_radius * (1.0 - t)) + (amp * profile_envelope)
            
#             # Keep your secondary sine wiggles for spatial clay texturing
#             current_radius += (amp * 0.3) * math.sin(t * freq * math.pi * 2)
            
#             # Map out coordinates on the horizontal XY plane with the updated winding angle
#             x = base_center.X + current_radius * math.cos(current_angle)
#             y = base_center.Y + current_radius * math.sin(current_angle)
            
#             strand_points.append(rg.Point3d(x, y, z))
            
#         # Ensure the last point snaps perfectly to the common top apex tip
#         strand_points[-1] = apex_point
        
#         # Create geometry and extract printing alignment frames
#         curve_geom = rg.Curve.CreateInterpolatedCurve(strand_points, 3)
#         curve_id = rs.AddInterpCurve(strand_points)
#         curves_out.append(curve_id)
        
#         if curve_geom:
#             t_params = curve_geom.DivideByCount(samples, True)
#             for t_val in t_params:
#                 rc, perp_plane = curve_geom.PerpendicularFrameAt(t_val)
#                 if rc:
#                     # --- CRITICAL ROBOT ALIGNMENT FIX ---
#                     # Keep the exact location on the curve (Origin) 
#                     # Keep the exact direction of travel (Normal/ZAxis)
#                     origin = perp_plane.Origin
#                     z_axis = perp_plane.ZAxis  # This is the path direction
                    
#                     # Choose World X as our primary guide vector
#                     world_guide = rg.Vector3d(1, 0, 0)
                    
#                     # If the curve travels straight along World X, switch guide to World Y 
#                     # to prevent mathematical breakdown (cross product with parallel vectors = 0)
#                     if abs(z_axis.X) > 0.98:
#                         world_guide = rg.Vector3d(0, 1, 0)
                    
#                     # Compute a clean local Y-axis perpendicular to both the path and our world guide
#                     local_y = rg.Vector3d.CrossProduct(z_axis, world_guide)
#                     local_y.Unitize()
                    
#                     # Compute the final corrected local X-axis to complete the perpendicular system
#                     local_x = rg.Vector3d.CrossProduct(local_y, z_axis)
#                     local_x.Unitize()
                    
#                     # Construct the newly locked target plane
#                     aligned_plane = rg.Plane(origin, local_x, local_y)
                    
#                     planes_out.append(aligned_plane)
                    
#     return curves_out, planes_out


def create_vertical_spatial_bulb(target_height=400.0, amp=40.0, freq=4.0, samples=60, num=3, radius=100, spiral_turns=0.5):
    """
    Generates robotic toolpaths where approach, spiral, and exit fluidly form a single curve 
    per strand, with tool orientation planes locked to the world X/Y grid.
    """
    start_points = []
    alpha = (2 * math.pi) / num
    
    # Generate symmetrical base anchor points automatically
    for n in range(num):
        x = math.sin(alpha * n) * radius
        y = math.cos(alpha * n) * radius
        z = 0 
        start_points.append(rg.Point3d(x, y, z))
        
    center_x = sum(p.X for p in start_points) / float(len(start_points))
    center_y = sum(p.Y for p in start_points) / float(len(start_points))
    center_z = sum(p.Z for p in start_points) / float(len(start_points))
    base_center = rg.Point3d(center_x, center_y, center_z)
    
    curves_out = []
    planes_out = [] 
    
    # Internal helper to maintain stable world-aligned robot orientations
    def align_plane_to_world(origin, z_axis):
        world_guide = rg.Vector3d(1, 0, 0)
        if math.fabs(z_axis.X) > 0.98:
            world_guide = rg.Vector3d(0, 1, 0)
        local_y = rg.Vector3d.CrossProduct(z_axis, world_guide)
        local_y.Unitize()
        local_x = rg.Vector3d.CrossProduct(local_y, z_axis)
        local_x.Unitize()
        return rg.Plane(origin, local_x, local_y)

    for p in start_points:
        dx = p.X - base_center.X
        dy = p.Y - base_center.Y
        
        start_radius = math.sqrt(dx**2 + dy**2)
        base_angle = math.atan2(dy, dx)
        
        # This will hold the entire seamless continuous toolpath sequence for this strand
        all_strand_points = []
        curve_points = []
        # ---------------------------------------------------------
        # PHASE 1: VERTICAL APPROACH POINTS (10cm Lift)
        # ---------------------------------------------------------
        # Straight vertical plunge coordinates down to the floor anchor point
        for step in [1, 2, 3]:
            approach_z = p.Z + (step * 40) 
            all_strand_points.append(rg.Point3d(p.X, p.Y, approach_z))
        
        curve_points.append(p)
        
        # ---------------------------------------------------------
        # PHASE 2: SPIRAL SECTIONS (With Apex Space Clearance)
        # ---------------------------------------------------------
        spiral_points = []
        for i in range(samples + 1):
            t = i / float(samples)
            z = base_center.Z + (t * target_height) + all_strand_points[-1].Z  # Offset by the last approach point's Z
            
            total_spiral_angle = spiral_turns * 2 * math.pi
            current_angle = base_angle + (t * total_spiral_angle)
            
            profile_envelope = math.sin(t * math.pi)
            
            # Enforce 23mm spacing ring clear zone at crown apex
            target_top_radius = 46
            tapered_radius = (start_radius * (1.0 - t)) + (target_top_radius * t)
            
            current_radius = tapered_radius + (amp * profile_envelope)
            current_radius += (amp * 0.3) * math.sin(t * freq * math.pi * 2)
            
            x = base_center.X + current_radius * math.cos(current_angle)
            y = base_center.Y + current_radius * math.sin(current_angle)
            
            spiral_points.append(rg.Point3d(x, y, z))
            
        curve_points.extend(spiral_points)
        
        # Build and add the main curve geometry to Rhino
        curve_geom = rg.Curve.CreateInterpolatedCurve(curve_points, 3)
        curve_id = rs.AddInterpCurve(curve_points)
        curves_out.append(curve_id)
        
        # Extract aligned printing planes along the main path
        if curve_geom:
            total_divisions = len(curve_points) * 2
            t_params = curve_geom.DivideByCount(total_divisions, True)
            
            for t_val in t_params:
                # Get the actual localized vector direction of the curve pathway
                tangent = curve_geom.TangentAt(t_val)
                pt = curve_geom.PointAt(t_val)
                
                # --- ANTIFLIP SECURITY SYSTEM ---
                # Check the tangent vector direction against the path trajectory.
                # During vertical approach, the curve goes down, so tangent points down (-Z).
                # During the spiral, the curve travels up, so tangent points up (+Z).
                # This guarantees plane coordinate frames never flip upside down randomly.
                aligned_p = align_plane_to_world(pt, tangent)
                planes_out.append(aligned_p)
        
        # ---------------------------------------------------------
        # PHASE 3: DYNAMIC EXIT PLANES (Targeting the Next Strand Start)
        # ---------------------------------------------------------
        if len(spiral_points) > 0:
            top_pt = spiral_points[-1]
            
            # Look ahead to locate the next target strand's start position
            next_index = (n + 1) % num
            next_base_pt = start_points[next_index]
            next_approach_start = rg.Point3d(next_base_pt.X, next_base_pt.Y, next_base_pt.Z + 100.0)
            
            # Vector pointing straight towards the next start point
            target_vector = next_approach_start - top_pt
            
            # Flatten completely to the horizontal plane to prevent diving or lifting
            exit_dir = rg.Vector3d(target_vector.X, target_vector.Y, 0.0)
            exit_dir.Unitize()
            
            # For the final strand, continue outward to safely clear the print area
            if n == num - 1:
                radial_vector = top_pt - base_center
                exit_dir = rg.Vector3d(-radial_vector.X, -radial_vector.Y, 0.0)
                exit_dir.Unitize()  
            
            # Generate stepping escape targets using consistent alignment logic
            for step in [1, 2]:
                exit_pt = top_pt + (exit_dir * (step * 150.0))
                
                # FIXED: We pass the exit direction vector directly into the uniform matrix helper
                # This guarantees that the X, Y, and Z orientations match the structural system
                # used during the print path, preventing robot joint tracking errors.
                exit_plane = align_plane_to_world(exit_pt, exit_dir)
                planes_out.append(exit_plane)
                    
    return curves_out, planes_out



def create_vertical_spatial_bulb(target_height=400.0, amp=40.0, freq=4.0, samples=60, num=3, radius=100, spiral_turns=0.5):
    """
    Generates spatially inclined robotic toolpaths. The paths lean outward in space 
    while keeping the tool orientation Z-axis locked perfectly vertical.
    """
    start_points = []
    alpha = (2 * math.pi) / num
    
    # Generate symmetrical base anchor points automatically
    for n in range(num):
        x = math.sin(alpha * n) * radius
        y = math.cos(alpha * n) * radius
        z = 0 
        start_points.append(rg.Point3d(x, y, z))
        
    center_x = sum(p.X for p in start_points) / float(len(start_points))
    center_y = sum(p.Y for p in start_points) / float(len(start_points))
    center_z = sum(p.Z for p in start_points) / float(len(start_points))
    base_center = rg.Point3d(center_x, center_y, center_z)
    
    curves_out = []
    planes_out = [] 
    
    # Internal helper to construct an upright plane locked to the World Z-axis
    def build_upright_travel_plane(origin, travel_direction, fallback_direction):
        z_axis = rg.Vector3d(0, 0, 1) # Forced vertical nozzle axis
        
        # Flatten the travel direction onto the horizontal XY plane
        x_axis = rg.Vector3d(travel_direction.X, travel_direction.Y, 0.0)
        
        if x_axis.Length < 0.001:
            x_axis = rg.Vector3d(fallback_direction.X, fallback_direction.Y, 0.0)
            
        x_axis.Unitize()
        
        # Calculate Y via cross product to keep the coordinate system squared
        y_axis = rg.Vector3d.CrossProduct(z_axis, x_axis)
        y_axis.Unitize()
        
        return rg.Plane(origin, x_axis, y_axis)

    for n in range(num):
        p = start_points[n]
        dx = p.X - base_center.X
        dy = p.Y - base_center.Y
        
        start_radius = math.sqrt(dx**2 + dy**2)
        base_angle = math.atan2(dy, dx)
        
        radial_fallback = p - base_center
        curve_points = []
        
        # Approach sequence (descending vertically)
        for step in [3, 2, 1]:
            approach_z = p.Z + (step * 33.3) 
            curve_points.append(rg.Point3d(p.X, p.Y, approach_z))
            
        curve_points.append(p)
            
        # Spiral structural sweep points
        spiral_points = []
        for i in range(samples + 1):
            t = i / float(samples)
            z = base_center.Z + (t * target_height)
            
            total_spiral_angle = spiral_turns * 2 * math.pi
            current_angle = base_angle + (t * total_spiral_angle)
            
            profile_envelope = math.sin(t * math.pi)
            
            # --- MODIFIED FOR OUTWARD INCLINATION ---
            # Instead of shrinking inward to 23mm, we maintain or increase the envelope space
            # to lean the strands outward as they gain vertical height.
            inclination_factor = 1.0 + (t * 0.4) # Adjust 0.4 to tilt more or less out
            tapered_radius = start_radius * inclination_factor
            
            current_radius = tapered_radius + (amp * profile_envelope)
            current_radius += (amp * 0.3) * math.sin(t * freq * math.pi * 2)
            
            x = base_center.X + current_radius * math.cos(current_angle)
            y = base_center.Y + current_radius * math.sin(current_angle)
            
            spiral_points.append(rg.Point3d(x, y, z))
            
        curve_points.extend(spiral_points)
        
        # Build and add the main curve geometry to Rhino
        curve_geom = rg.Curve.CreateInterpolatedCurve(curve_points, 3)
        curve_id = rs.AddInterpCurve(curve_points)
        curves_out.append(curve_id)
        
        # Extract perfectly vertical toolplanes along the active printing path
        if curve_geom:
            total_divisions = len(curve_points) * 2
            t_params = curve_geom.DivideByCount(total_divisions, True)
            
            for t_val in t_params:
                tangent = curve_geom.TangentAt(t_val)
                pt = curve_geom.PointAt(t_val)
                
                printing_plane = build_upright_travel_plane(pt, tangent, radial_fallback)
                planes_out.append(printing_plane)
        
        # ---------------------------------------------------------
        # PHASE 3: DYNAMIC EXIT PLANES
        # ---------------------------------------------------------
        if len(spiral_points) > 0:
            top_pt = spiral_points[-1]
            
            next_index = (n + 1) % num
            next_base_pt = start_points[next_index]
            next_approach_start = rg.Point3d(next_base_pt.X, next_base_pt.Y, next_base_pt.Z + 100.0)
            
            target_vector = next_approach_start - top_pt
            exit_dir = rg.Vector3d(target_vector.X, target_vector.Y, 0.0)
            
            if n == num - 1:
                exit_dir = top_pt - base_center
                exit_dir.Z = 0.0
            
            exit_dir.Unitize()
            
            for step in [1, 2]:
                exit_pt = top_pt + (exit_dir * (step * 50.0))
                exit_plane = build_upright_travel_plane(exit_pt, exit_dir, radial_fallback)
                planes_out.append(exit_plane)
                    
    return curves_out, planes_out

# --------------------------------------------------
# Graph construction
# --------------------------------------------------

def build_graph(radius, num):
    """Construct a NodeGraph from cell records and edge relations."""
    g = Graph()
    primal_mesh = exp_03(radius, num)
    structure = primal_mesh.edges(data=True)
    for edges, _ in structure:
        u, v = edges
        g.add_edge(u, v)   
    g.add_edge(0, 2) 
    return g, primal_mesh


# ------------------------------------------------------------------------
# Main API
# ------------------------------------------------------------------------

# def build_mesh_graph(
#     div_x=None,
#     div_y=None,
#     debug=False
# ):
#     """
#     Build a hierarchical mesh graph.
    
#     Returns
#     -------
#     NodeGraph
#         The constructed graph with all nodes and edges.
#     """
#     # Create mesh
#     mesh = exp_03()
#     mesh_nodes = mesh.vertex
#     # Build graph
#     ng = build_graph(mesh_nodes)
    
#     return ng

def print_path(radius, num):
    # Build Mesh Graph
    mg, mesh = build_graph(radius, num)
    
    V = mesh.number_of_vertices()
    print(V)
    E = mesh.number_of_edges()
    print(E)
    F = mesh.number_of_faces()
    print(F)
    mesh.euler() == V - E + F
    print(mesh.euler())
    
    return mg
    