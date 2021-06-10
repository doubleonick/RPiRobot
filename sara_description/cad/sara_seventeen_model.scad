module cube_base(width, length, height)
{
    cube(size = [width, length, height], center = true);
}

module cylinder_base(radius, height, roundness)
{
    cylinder(r = radius, h = height, $fn = roundness, center = true);
}

module cone_base(radius1, radius2, height, roundness)
{
    cylinder(r1 = radius1, r2 = radius2, h = height, $fn = roundness, center = true);
}

module sphere_base(radius, roundness)
{
    sphere(r = radius, $fn = roundness, center = true);
}
/****************************************************************/
in_to_mm      = 25.4;
frame_outer_x = 17 * in_to_mm;
frame_outer_y = 8.75 * in_to_mm;
frame_outer_z = 8.625 * in_to_mm;

frame_innerx_x = frame_outer_x - (2 * in_to_mm);
frame_innerx_y = frame_outer_y;
frame_innerx_z = frame_outer_z - (2.125 * in_to_mm);

frame_innery_x = frame_outer_x;
frame_innery_y = frame_outer_y - (2 * in_to_mm);
frame_innery_z = frame_outer_z - (2.125 * in_to_mm);

module sensor_modular_unit()
{
    difference()
    {
        cube_base(frame_outer_x, frame_outer_y, frame_outer_z);
        cube_base(frame_innerx_x, frame_innerx_y, frame_innerx_z);
        cube_base(frame_innery_x, frame_innery_y, frame_innery_z);
    }
    
    //add cube that is 4 3/8" x 4 3/8" x 5/8" for base of ouster
    translate([0, 0, (frame_outer_z * 0.5) + (4.31 * in_to_mm)])
    cube_base(4.375 * in_to_mm, 4.375 * in_to_mm, 0.625 * in_to_mm); 
    
    translate([0, 0, frame_outer_z * 0.5])
    {
        rotate([90, 0, 0])
        translate([-50, 0, -50])
        scale([25.25, 25.25, 25.25])
        import("LIDAR_Ouster_OS1_64Line_Mount.STL");
    }
    translate([0, frame_outer_y * 0.5 - 27, frame_outer_z * 0.5])
    {
        rotate([90, 0, 180])
        translate([-50, 0, 6.5])
        import("front_realsense_mount.STL");
    }
    translate([8.75 * 0.5 * in_to_mm, frame_outer_y * 0.5 -15, frame_outer_z * 0.5])
    {
        rotate([90, 0, 0])
        translate([-30, 0, -15])
        import("Flir FL2-020-R0 camera mount block.STL");
    }
    mirror([1, 0, 0])
    translate([8.75 * 0.5 * in_to_mm, frame_outer_y * 0.5 -15, frame_outer_z * 0.5])
    {
        rotate([90, 0, 0])
        translate([-30, 0, -15])
        import("Flir FL2-020-R0 camera mount block.STL");
    }
}

rotate([0, 0, -90])
scale([0.001968504/2, 0.001968504/2, 0.001968504/2])
translate([0, 0, frame_outer_z * 0.5 - 30])
sensor_modular_unit();

/************************************/
/************************************/
/************************************/
module soffos_modular_unit()
{
    rotate([0, 180, 180])
    translate([-frame_outer_y * 0.5, -frame_outer_x * 0.5, -(frame_outer_z * 1.5 - 36)])
    import("/home/user1/dcist_ws/src/sara_description/cad/SARA sensor modular unit for Husky.STL");
//
//    //add cube that is 4 3/8" x 4 3/8" x 5/8" for base of ouster
//    translate([0, 0, (frame_outer_z - 18) + ((4) * in_to_mm) - 4.5])
//    cube_base(4.375 * in_to_mm, 4.375 * in_to_mm, 0.625 * in_to_mm);

    
}
//scale([0.001968504/2, 0.001968504/2, 0.001968504/2])
//translate([-70, 0, (12.625 * in_to_mm * 0.5) - 22])
//cube_base(100, 150, 13.25 * in_to_mm);
//12.625 is calculated frame + ouster mount height based on physical measurement of each
//scale([0.001968504/2, 0.001968504/2, 0.001968504/2])
//soffos_modular_unit();