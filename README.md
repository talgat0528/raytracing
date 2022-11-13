# raytracing
ray tracer implementation in C.
Ray tracing is a fundamental rendering algorithm. It is commonly used for animations and archi- tectural simulations, in which the quality of the created images is more important than the time it takes to create them. Here I implemented a basic ray tracer that simulates the propagation of light in real world.
# How to run
Navigate to the source directory and run the following command in your terminal:
$make
After the code has been compiled, run your executable via command:
$./raytracer ../sample_scenes_v4/inputs/bunny.xml
The executable takes XML scene file as an argument, e.g bunny.xml
Note that bunny.xml is one of the sample input files and it is given as an example
The resulting image will be saved in the source folder in the PPM format
# Specifications and details
- The scene file may contain multiple camera configurations. The raytracer renders as many images as the number of cameras.
- Used Blinn-Phong shading model for the specular shading computations.
- Implemented two types of light sources: point and ambient. There may be multiple point light sources and a single ambient light. The values of these lights will be given as (R, G, B) color triplets that are not restricted to [0, 255] range. So, any pixel color value that is calculated by shading computations and is greater than 255 is clamped to 255 and rounded to the nearest integer before being written to the output PPM file.
- Point lights are defined by their intensity (power per unit solid angle). The irradiance due to such a light source falls off as inversely proportional to the squared distance from the light source. To simulate this effect, I computed the irradiance at a distance of d from a point light as: E(d) = I/(d^2), where I is the original light intensity (a triplet of RGB values given in the XML file) and E(d) is the irradiance at distance d from the light source.
- There may be different numbers of materials, vertices, triangles, spheres, lights, and cameras.
- Used the -O3 option while compiling the code for optimization. This itself will provide a huge performance improvement.
- Used Back-face culling. It is a method used to accelerate the ray-scene intersections by not computing intersections with triangles whose normals are pointing away from the camera. Its implementation is simple and done by calculating the dot product of the ray direction with the normal vector of the triangle. If the sign of the result is positive, then that triangle is ignored.
- Pre-computed anything that would be used multiple times and saved these results. It is strongly advised to do so because functions that run for each pixel will slow down the performance. So, anything that gets implemented multiple times and returns the same result can be pre-computed. For example, you can pre-compute the normals of the triangles and save it in your triangle data structure when you read the input file.
