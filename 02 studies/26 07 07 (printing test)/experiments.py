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

def create_print_circular_base(start_point, base_radius, base_curve_resolution):
    """Create circular base for the print"""
    if base_curve_resolution is None:
        base_curve_resolution = 20
    base_planes = []
    
    circumference = 2 * math.pi * base_radius
    num = int(circumference / base_curve_resolution)
    # alpha = (2 * math.pi) / num
    
    # for n in range(num):
    #     x = math.sin(alpha * n) * base_radius + start_point.X
    #     y = math.cos(alpha * n) * base_radius + start_point.Y
    #     z = 0 + start_point.Z
    #     base_planes.append(rg.Plane(rg.Point3d(x, y, z), rg.Vector3d(0, 0, 1)))
    
    for i in range(num):
        t = (i / float(num)) * (1 * 2 * math.pi)
        
        current_radius = base_radius
        
        x = current_radius * math.cos(t) + start_point.X
        y = current_radius * math.sin(t) + start_point.Y
        z = (t / (2 * math.pi)) + start_point.Z
        
        base_planes.append(rg.Plane(rg.Point3d(x, y, z), rg.Vector3d(0, 0, 1)))
    
    return base_planes

def create_print_entry(start_point, entry_safe_distance):
    """Create entry planes of the print"""
    entry_planes = []
    
    # entry_point = curve_planes[0].Origin
    entry_plane = rg.Plane(start_point, rg.Vector3d(1, 0, 0), rg.Vector3d(0, 1, 0))
    entry_planes.append(entry_plane)
    entry_planes.append(rg.Plane(rg.Point3d(start_point.X, start_point.Y, start_point.Z + entry_safe_distance), rg.Vector3d(1, 0, 0), rg.Vector3d(0, 1, 0)))
    
    return entry_planes

def create_print_exit(end_plane, exit_direction, exit_safe_distance):
    """Create exit planes of the print"""
    if exit_direction == None or exit_safe_distance == None:
        exit_safe_distance = 250
        exit_direction = rg.Vector3d(exit_safe_distance, 0, 0)

    exit_planes = []
    exit_planes.append(rg.Plane(rg.Point3d(end_plane.Origin.X, end_plane.Origin.Y  + 20, end_plane.Origin.Z), rg.Vector3d(1, 0, 0), rg.Vector3d(0, 0, -1)))
    exit_planes.append(rg.Plane(rg.Point3d(end_plane.Origin.X, end_plane.Origin.Y  + exit_safe_distance, end_plane.Origin.Z), rg.Vector3d(1, 0, 0), rg.Vector3d(0, 0, -1)))

    return exit_planes
    

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

def create_trig_curve(
    start_point,
    type="sine", 
    wavelength=10.0, 
    amplitude=2.0, 
    num_cycles=2, 
    curve_resolution=20, 
    base_radius=100.0,
    safe_distance=100.0,
    coef=.04,
    debug=False,
    planar=False
    ):
    """
    Generates a Sine or Cosine curve in Rhino and extracts manufacturing planes.
    
    Parameters:
    - type: "sine" or "cosine"
    - wavelength: The length of one complete wave cycle along the X-axis
    - amplitude: The peak height of the wave along the Y-axis
    - num_cycles: How many full waves to draw
    - curve_resolution: Controls the smoothness of the curve
    - safe_distance: Distance to keep from the curve for safe printing
    
    Returns:
    - tuple: (curve_id, planes_list)
    """
    points = []
    curve_planes = []
    planes_out = []
    
    # Calculate total points to plot
    total_samples = int(num_cycles * curve_resolution)
    total_length = wavelength * num_cycles
    
    # 1. Generate print base
    base_planes = create_print_circular_base(start_point, base_radius, base_curve_resolution=curve_resolution)
    planes_out.extend(base_planes)
    
    # 2. Generate entry planes 
    entry_planes = create_print_entry(start_point, entry_safe_distance=safe_distance)
    planes_out.extend(entry_planes)
    
    entry_start_point = entry_planes[-1].Origin
    
    # 3. Generate the trig curve points
    for i in range(total_samples + 1):
        # Calculate X coordinate
        x = (i / float(total_samples)) * total_length
        
        # Calculate the angle in radians based on the current X position
        angle = (2 * math.pi * x) / wavelength
        
        # Calculate Y coordinate based on type
        if type.lower() == "cosine":
            y = amplitude * math.cos(angle) * i * coef
        else:
            y = amplitude * math.sin(angle) * i * coef
            
        # Z is kept at 0 for a flat 2D curve on the XY plane
        z = 0
        
        points.append(rg.Point3d(z + entry_start_point.X, y + entry_start_point.Y, x + entry_start_point.Z + 20))
    
    # 4. Create and add the interpolated curve to the Rhino Document
    curve_geom = rg.Curve.CreateInterpolatedCurve(points, 3)
    
    if curve_geom:
            t_params = curve_geom.DivideByCount(total_samples, True)
            for t_val in t_params:
                rc, perp_plane = curve_geom.PerpendicularFrameAt(t_val)
                if rc:
                    origin = perp_plane.Origin
                    z_axis = perp_plane.ZAxis  # This is the path direction
                    # Choose World X as our primary guide vector
                    world_guide = rg.Vector3d(1, 0, 0)
                    # If the curve travels straight along World X, switch guide to World Y
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
                    if planar == True:
                        aligned_plane = rg.Plane(origin, rg.Vector3d(1, 0, 0), rg.Vector3d(0, 1, 0))
                    
                    curve_planes.append(aligned_plane)
    
    planes_out.extend(curve_planes)
    
    # 5. Generate exit planes
    exit_planes = create_print_exit(curve_planes[-1], exit_direction=None, exit_safe_distance=None)
    planes_out.extend(exit_planes)
    
    if debug:
        return points, base_planes, curve_geom, curve_planes, planes_out
    
    return curve_geom, planes_out

def create_trig_spiral(
    start_point, 
    amplitude=2.0,
    pitch=50.0, 
    num_cycles=2, 
    curve_resolution=20, 
    base_radius=100.0,
    safe_distance=100.0,
    coef=.04,
    planar=False,
    debug=False
    ):
    """
    Generates a Sine or Cosine curve in Rhino and extracts manufacturing planes.
    
    Parameters:
    - type: "sine" or "cosine"
    - wavelength: The length of one complete wave cycle along the X-axis
    - amplitude: The peak height of the wave along the Y-axis
    - num_cycles: How many full waves to draw
    - curve_resolution: Controls the smoothness of the curve
    - safe_distance: Distance to keep from the curve for safe printing
    
    Returns:
    - tuple: (curve_id, planes_list)
    """
    points = []
    curve_planes = []
    planes_out = []
    
    # Calculate total points to plot
    circumference = 2 * math.pi * base_radius
    num = int(circumference / curve_resolution)
    total_samples = int(num_cycles * (num))
    print(num)
    print(total_samples)
    # total_samples = int(num_cycles * curve_resolution)
    
    # 1. Generate print base
    base_planes = create_print_circular_base(start_point, base_radius, base_curve_resolution=curve_resolution)
    planes_out.extend(base_planes)
    
    # 3. Generate the spiral curve points    
    for i in range(total_samples):
        t = (i / float(total_samples)) * (num_cycles * 2 * math.pi)
        
        current_radius = base_radius
        
        x = current_radius * math.cos(t)
        y = current_radius * math.sin(t)
        z = (t / (2 * math.pi)) * pitch + safe_distance 
        
        points.append(rg.Point3d(x + start_point.X, y + start_point.Y, z + start_point.Z))
    
    # 4. Create and add the interpolated curve to the Rhino Document
    curve_geom = rg.Curve.CreateInterpolatedCurve(points, 3)
            
    if curve_geom:
            t_params = curve_geom.DivideByCount(total_samples, True)
            for t_val in t_params:
                rc, perp_plane = curve_geom.PerpendicularFrameAt(t_val)
                if rc:
                    origin = perp_plane.Origin
                    z_axis = perp_plane.ZAxis  # This is the path direction
                    # Choose World X as our primary guide vector
                    world_guide = rg.Vector3d(1, 0, 0)
                    # If the curve travels straight along World X, switch guide to World Y
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
                    if planar == True:
                        aligned_plane = rg.Plane(origin, rg.Vector3d(1, 0, 0), rg.Vector3d(0, 1, 0))
                    
                    curve_planes.append(aligned_plane)
    
    planes_out.extend(curve_planes)
    
    # 5. Generate exit planes
    exit_planes = create_print_exit(curve_planes[-1], exit_direction=None, exit_safe_distance=None)
    planes_out.extend(exit_planes)
    
    if debug:
        return points, base_planes, curve_geom, curve_planes, planes_out
    
    return curve_geom, planes_out, points, base_planes



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
    