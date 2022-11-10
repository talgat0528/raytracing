#include <iostream>
#include "parser.h"
#include "ppm.h"
#include <cmath>
using namespace std;
typedef unsigned char RGB[3];
class Vec3f {
    public:
        float x, y, z;
    public:
        Vec3f(){
            x = 0;
            y = 0;
            z = 0;
        }
        Vec3f(float _x,float _y, float _z) : x(_x), y(_y), z(_z) {}
        Vec3f multS(float const &s) {
            Vec3f result;
            result.x = x * s;
            result.y = y * s;
            result.z = z * s;
            return result;
        }
        Vec3f operator+(Vec3f const &obj) {
            Vec3f result;
            result.x = x + obj.x;
            result.y = y + obj.y;
            result.z = z + obj.z;
            return result;
        }
        Vec3f operator-(Vec3f const &obj) {
            Vec3f result;
            result.x = x - obj.x;
            result.y = y - obj.y;
            result.z = z - obj.z;
            return result;
        }
        Vec3f operator-(parser::Vec3f const &obj) {
            Vec3f result;
            result.x = x - obj.x;
            result.y = y - obj.y;
            result.z = z - obj.z;
            return result;
        }
        float operator*(Vec3f const &obj) {
            return x * obj.x + y * obj.y + z * obj.z;
        }
        Vec3f cross(Vec3f const &obj) {
            Vec3f result;
            result.x = y*obj.z - z*obj.y;
            result.y = z*obj.x - x*obj.z;
            result.z = x*obj.y - y*obj.x;
            return result;
        }
        Vec3f semiDot(parser::Vec3f const &obj) {
            Vec3f result;
            result.x = x*obj.x;
            result.y = y*obj.y;
            result.z = z*obj.z;
            return result;
        }
        Vec3f semiDot(Vec3f const &obj) {
            Vec3f result;
            result.x = x*obj.x;
            result.y = y*obj.y;
            result.z = z*obj.z;
            return result;
        }
        Vec3f& normalize() {
            float magnitude;
            magnitude = sqrt(pow(x,2) + pow(y,2) + pow(z,2));
            x = x/magnitude;
            y = y/magnitude;
            z = z/magnitude;
            return *this;
        }
        Vec3f& clamp(){
            if(x>255) x=255;
            if(y>255) y=255;
            if(z>255) z=255;
            return *this;
        }
        Vec3f& operator=(const Vec3f &obj){
            x = obj.x;
            y = obj.y;
            z = obj.z;
            return *this;
        }
        Vec3f& operator=(const parser::Vec3f &obj){
            x = obj.x;
            y = obj.y;
            z = obj.z;
            return *this;
        }
        float getMagnitude() {
            return sqrt(pow(x,2)+pow(y,2)+pow(z,2));
        }
        float getX() {
            return x;
        }
        float getY() {
            return y;
        }
        float getZ() {
            return z;
        }
};
class Ray {

    public:
        Vec3f origin;
        Vec3f direction;
    public:
        Ray(){
            origin.x = origin.y = origin.z = 0;
            direction.x = direction.y = direction.z = 0;
        }
        Ray(const Ray &obj){
            origin = obj.origin;
            direction = obj.direction;
            direction.normalize();
        }
        Ray(int i,int j, float su_right_part, float sv_right_part, Vec3f q, Vec3f u, Vec3f v, Vec3f e) {
            float su, sv;
            su = (i + 0.5)*su_right_part;
            sv = (j + 0.5)*sv_right_part;
            Vec3f s = q + u.multS(su) + v.multS(-sv);
            origin = e;
            direction = (s - e);
            direction.normalize();

        }
        Ray(Vec3f _origin, Vec3f _direction) {
            origin = _origin;
            direction = _direction;
            direction.normalize();
        }
        Ray& operator=(const Ray &obj){
            origin = obj.origin;
            direction = obj.direction;
            direction.normalize();
            return *this;
        }

        Vec3f getOrigin() {
            return origin;
        }
        Vec3f getDirection() {
            return direction;
        }
};
float det3x3(float a, float b, float c, float d, float e, float f, float g, float h, float i){
    return a*(e*i-h*f) + b*(g*f-d*i) + c*(d*h-e*g);
}
float intersectTriangle(Ray & r, int i,int j, bool triangle,std::vector<parser::Triangle> & triangles,std::vector<parser::Vec3f> & vertices,std::vector<parser::Mesh> & meshes) {
    Vec3f a,b,c;


    if(triangle) {
        a = vertices[triangles[i].indices.v0_id-1];
        b = vertices[triangles[i].indices.v1_id-1];
        c = vertices[triangles[i].indices.v2_id-1];
    }
    else {
        a = vertices[meshes[i].faces[j].v0_id-1];
        b = vertices[meshes[i].faces[j].v1_id-1];
        c = vertices[meshes[i].faces[j].v2_id-1];
    }

    float A = det3x3(a.x - b.x, a.y - b.y, a.z-b.z, a.x-c.x, a.y-c.y, a.z-c.z, r.direction.x, r.direction.y,r.direction.z);
    float beta = det3x3(a.x - r.origin.x, a.y - r.origin.y, a.z-r.origin.z, a.x-c.x, a.y-c.y, a.z-c.z, r.direction.x, r.direction.y,r.direction.z)/A;
    float gamma = det3x3(a.x - b.x, a.y - b.y, a.z-b.z, a.x-r.origin.x, a.y-r.origin.y, a.z-r.origin.z, r.direction.x, r.direction.y,r.direction.z)/A;
    float t = det3x3(a.x - b.x, a.y - b.y, a.z-b.z, a.x-c.x, a.y-c.y, a.z-c.z, a.x-r.origin.x, a.y-r.origin.y,a.z-r.origin.z)/A;
    if(beta+gamma<=1 && beta>=0 && gamma >=0){
        return t;
    }
    else {
        return -1;
    }
}
float intersectSphere(Ray & r, int i,std::vector<parser::Vec3f> & vertices,std::vector<parser::Sphere> & spheres)
{
	float A,B,C; //constants for the quadratic equation
	
	float delta;
	Vec3f c;

	
    float radius;
	c = vertices[spheres[i].center_vertex_id-1];
    radius = spheres[i].radius;
	
	float t,t1,t2;
	
	C = (r.origin.x-c.x)*(r.origin.x-c.x)+(r.origin.y-c.y)*(r.origin.y-c.y)+(r.origin.z-c.z)*(r.origin.z-c.z)-radius*radius;

	B = 2*r.direction.x*(r.origin.x-c.x)+2*r.direction.y*(r.origin.y-c.y)+2*r.direction.z*(r.origin.z-c.z);
	
	A = r.direction.x*r.direction.x+r.direction.y*r.direction.y+r.direction.z*r.direction.z;
	
	delta = B*B-4*A*C;
	
	if (delta<0) return -1;
	else if (delta==0)
	{
		t = -B / (2*A);
	}
	else
	{
		delta = sqrt(delta);
		A = 2*A;
		t1 = (-B + delta) / A;
		t2 = (-B - delta) / A;
				
		if (t1<t2) t=t1; else t=t2;
	}
	
	return t;
}


Vec3f computeColor(Ray ray, parser::Scene & scene, int max_recursion_depth, bool inRecursion,std::vector<parser::Triangle> & triangles,std::vector<parser::Vec3f> & vertices,std::vector<parser::Mesh> & meshes,std::vector<parser::Sphere> & spheres, std::vector<parser::Material> & materials) {
	int i,j;
    j = 0;
	Vec3f c;
	double minT = 90000; 
	double t;
	Vec3f L,N;
	Vec3f P;
	int minI,minJ;
    int numSpheres = spheres.size();
    int numTriangles = triangles.size();
    int numMeshes = meshes.size();
    char intersectsWith = 'n';
    if(inRecursion) {
        c.x = 0;
        c.y = 0;
        c.z = 0;
    }
    else {
        c.x = scene.background_color.x; // CHANGE
        c.y = scene.background_color.y;
        c.z = scene.background_color.z;
    }
    Vec3f ambientLight;
    Vec3f light,V,HalfV;
    int numLights = scene.point_lights.size();
    Vec3f intensity;
    float distanceOfLight;
    float phong;
    Vec3f diffuse,specular,objectBeforeLight,mirrorCoeff;
    bool blocking = false;
    bool reflective = false;
    int k=0;
    Vec3f epsVector;
    Ray S; 
    ambientLight = scene.ambient_light;
    minI = minJ = -1;
	for (i = 0; i< numSpheres; i++)
	{  
		t = intersectSphere(ray,i,vertices,spheres); 
		if (t<minT && t>0)
		{
			c = ambientLight.semiDot(materials[spheres[i].material_id-1].ambient);
            //c.x=c.y=c.z=0;
			minI = i;
			minT = t;
            intersectsWith = 's';
            
		}
	}
	for (i = 0; i< numTriangles; i++)
	{  
		t = intersectTriangle(ray,i,j,true,triangles,vertices,meshes);
		if (t<minT && t>0)
		{
			c = ambientLight.semiDot(materials[triangles[i].material_id-1].ambient); 
            //c.x=c.y=c.z=0;
			minI = i;
			minT = t;
            intersectsWith = 't';
		}
	}
    for (i = 0; i< numMeshes; i++)
	{   int numFaces = meshes[i].faces.size();
        for(j = 0; j < numFaces;j++){
            t = intersectTriangle(ray,i,j,false,triangles,vertices,meshes);
            if (t<minT && t>0)
            {
                c = ambientLight.semiDot(materials[meshes[i].material_id-1].ambient); 
                //c.x=c.y=c.z=0;
                minI = i;
                minJ = j;
                minT = t;
                intersectsWith = 'm';
                
            }
        }

	}
    //DIFFUSE + SPECULAR + SHADOWS
    P = ray.origin + ray.direction.multS(minT);
    V = ray.direction.multS(-1);
    V.normalize();
    if(intersectsWith=='s'){
            diffuse = materials[spheres[minI].material_id-1].diffuse;
            specular = materials[spheres[minI].material_id-1].specular;
            reflective = materials[spheres[minI].material_id-1].is_mirror;
            
            phong = materials[spheres[minI].material_id-1].phong_exponent;
            N = P - vertices[spheres[minI].center_vertex_id-1];
            N.normalize();
            epsVector = N.multS(scene.shadow_ray_epsilon);
            for(i = 0; i<numLights;i++) {
                light = scene.point_lights[i].position;
                L =  light - P;


                distanceOfLight = sqrt(pow(light.x-P.x,2)+pow(light.y-P.y,2)+pow(light.z-P.z,2));
                L.normalize();
                
                //STARTS CHECKING SHADOWS-----------------------
                
                S.origin = P+epsVector;
                S.direction = L;
                
 
                blocking = false;
                for(j = 0; j<numSpheres; j++) {
                   
                        t = intersectSphere(S,j,vertices,spheres);
                        if(t>0) {
                            objectBeforeLight = S.direction.multS(t);
                            
                            if(objectBeforeLight.getMagnitude() < distanceOfLight) {
                                blocking = true;
                                break;
                            }
                        }
                   
                }
                if(blocking) continue;
                for(j = 0; j<numTriangles; j++) {
                 
                        t = intersectTriangle(S,j,k,true,triangles,vertices,meshes);
                        if(t>0) {
                            objectBeforeLight = S.direction.multS(t);
                            
                            if(objectBeforeLight.getMagnitude() < distanceOfLight) {
                                blocking = true;
                                break;
                            }
                        }
         
                }
                if(blocking) continue;
                for(j = 0; j<numMeshes; j++) {
                    int numFaces = meshes[j].faces.size();
                    
                    for(k=0;k<numFaces;k++){
                      
                            t = intersectTriangle(S,j,k,false,triangles,vertices,meshes);
                            if(t>0) {
                                objectBeforeLight = S.direction.multS(t);
                                if(objectBeforeLight.getMagnitude() < distanceOfLight) {
                                    blocking = true;
                                    break;
                                }
                            }
                
                    }
                    if(blocking) break;
                }
                if(blocking) continue;

                // ENDS CHECKING SHADOWS ----------




                //COMPUTES COLOR
                
                HalfV = V+L;
                HalfV.normalize();
                
                intensity = scene.point_lights[i].intensity;      
                intensity = intensity.multS(1/(distanceOfLight*distanceOfLight));
                if(L*N>0){ 
                    c = c+ diffuse.semiDot(intensity.multS(L*N));
                }
                if(HalfV*N>0) {
                    c = c + specular.semiDot(intensity.multS(pow(HalfV*N,phong)));
                }
            }


            // COMPUTES REFLECTANCE
            if(reflective && max_recursion_depth>0) {

                Vec3f W0,Wr,W0neg;
                W0 = ray.direction.multS(-1);
                W0.normalize();       
                Wr = W0.multS(-1) + N.multS(2).multS(N*W0);
                Wr.normalize();
                S.direction = Wr;
                mirrorCoeff = materials[spheres[minI].material_id-1].mirror;
                c = c+ computeColor(S,scene,max_recursion_depth-1,true,triangles,vertices,meshes,spheres,materials).semiDot(mirrorCoeff);
            }

    }
    else if(intersectsWith=='t' || intersectsWith=='m') {
        Vec3f v0,v1,v2;
        if(intersectsWith == 't') {
            diffuse = materials[triangles[minI].material_id-1].diffuse;
            specular = materials[triangles[minI].material_id-1].specular;
            reflective = materials[triangles[minI].material_id-1].is_mirror;
            mirrorCoeff = materials[triangles[minI].material_id-1].mirror;
            phong = materials[triangles[minI].material_id-1].phong_exponent;

            v0 = vertices[triangles[minI].indices.v0_id-1];
            v1 = vertices[triangles[minI].indices.v1_id-1];
            v2 = vertices[triangles[minI].indices.v2_id-1];
        }
        else {
            diffuse = materials[meshes[minI].material_id-1].diffuse;
            specular = materials[meshes[minI].material_id-1].specular;
            reflective = materials[meshes[minI].material_id-1].is_mirror;
            mirrorCoeff = materials[meshes[minI].material_id-1].mirror;
            phong = materials[meshes[minI].material_id-1].phong_exponent;

            v0 = vertices[meshes[minI].faces[minJ].v0_id-1];
            v1 = vertices[meshes[minI].faces[minJ].v1_id-1];
            v2 = vertices[meshes[minI].faces[minJ].v2_id-1];
        }

        N = (v1-v0).cross(v2-v0);
        N.normalize();
        epsVector = N.multS(scene.shadow_ray_epsilon);
        for(i=0;i<numLights;i++) {
            light = scene.point_lights[i].position;
            L =  light - P;
            distanceOfLight = sqrt(L.x*L.x+L.y*L.y+L.z*L.z);

            //STARTS CHECKING SHADOWS-----------------------
            
            S.origin = P+epsVector;
            S.direction = L;
            S.direction.normalize();
            blocking = false;
            for(j = 0; j<numSpheres; j++) {
                
                    t = intersectSphere(S,j,vertices,spheres);
                    if(t>0) {
                        objectBeforeLight = S.direction.multS(t);
                        objectBeforeLight.getMagnitude();
                    
                        if(objectBeforeLight.getMagnitude() < distanceOfLight) {
                            blocking = true;
                            break;
                        }
                    }
              
            }
            if(blocking) continue;
            for(j = 0; j<numTriangles; j++) {
              
                    t = intersectTriangle(S,j,k,true,triangles,vertices,meshes);
                    if(t>0) {
                        objectBeforeLight = S.direction.multS(t);
                        if(objectBeforeLight.getMagnitude() < distanceOfLight) {
                            blocking = true;
                            break;
                        }
                    }
       
            }
            if(blocking) continue;
            for(j = 0; j<numMeshes; j++) {
                int numFaces = meshes[j].faces.size();
                for(k=0;k<numFaces;k++){
              
                        t = intersectTriangle(S,j,k,false,triangles,vertices,meshes);
                        if(t>0) {
                            objectBeforeLight = S.direction.multS(t);
                            if(objectBeforeLight.getMagnitude() < distanceOfLight) {
                                blocking = true;
                                break;
                            }
                        }

                }
                if(blocking) break;
            }
            if(blocking) continue;

            // ENDS CHECKING SHADOWS ----------/

            //COMPUTES COLOR 
            HalfV = V+L;
            HalfV.normalize();       
            L.normalize();
            intensity = scene.point_lights[i].intensity; 
            intensity = intensity.multS(1/(distanceOfLight*distanceOfLight));
            if(L*N>0) {
                c = c+ diffuse.semiDot(intensity.multS(L*N));
            }
            if(HalfV*N>0) {
                c = c + specular.semiDot(intensity.multS(pow(HalfV*N,phong)));
            }
        }
                   // COMPUTES REFLECTANCE

        if(reflective && max_recursion_depth>0) {
            Vec3f W0,Wr,W0neg;
            W0 = ray.direction.multS(-1);
            W0.normalize();       
            Wr = W0.multS(-1) + N.multS(2).multS(N*W0);
            Wr.normalize();
            S.direction = Wr;
            c = c+ computeColor(S,scene,max_recursion_depth-1,true,triangles,vertices,meshes,spheres,materials).semiDot(mirrorCoeff);
        }
    }

	return c;
}
int main(int argc, char* argv[])
{
    
    // Sample usage for reading an XML scene file
    parser::Scene scene;

    scene.loadFromXml(argv[1]);


    // CODE BELOW
    std::vector<parser::Triangle> & triangles = scene.triangles;
    std::vector<parser::Vec3f> & vertices = scene.vertex_data;
    std::vector<parser::Mesh> & meshes = scene.meshes;
    std::vector<parser::Sphere> & spheres = scene.spheres;
    std::vector<parser::Material> & materials = scene.materials;
    


    // raytracing loop, DO FOR ALL CAMERAS!!!
    int c;
    for(c = 0;c<scene.cameras.size();c++){
        int nx = scene.cameras[c].image_width;
    int ny = scene.cameras[c].image_height;
        unsigned char* image = new unsigned char [nx * ny * 3];

    int i = 0;
    for (int y = 0; y < ny; ++y)
    {
        for (int x = 0; x < nx; ++x)
        {
            image[i++] = scene.background_color.x;
            image[i++] = scene.background_color.y;
            image[i++] = scene.background_color.z;
        }
    }
        float su_right_part = (scene.cameras[c].near_plane.y - scene.cameras[c].near_plane.x)/nx;
        float sv_right_part = (scene.cameras[c].near_plane.w - scene.cameras[c].near_plane.z)/ny;
        Vec3f gaze(scene.cameras[c].gaze.x,scene.cameras[c].gaze.y,scene.cameras[c].gaze.z);
        gaze.normalize();
        Vec3f w = gaze.multS(-1);
        w.normalize();
        Vec3f v(scene.cameras[c].up.x,scene.cameras[c].up.y,scene.cameras[c].up.z);
        v.normalize();
        Vec3f u = v.cross(w);
        u.normalize();
        Vec3f e(scene.cameras[c].position.x,scene.cameras[c].position.y,scene.cameras[c].position.z);
        Vec3f m = e + gaze.normalize().multS(scene.cameras[c].near_distance);
        Vec3f q = m + u.multS(scene.cameras[c].near_plane.x) + v.multS(scene.cameras[c].near_plane.w);

        i = 0;
        for (int y = 0; y < ny; ++y)
        {
            for (int x = 0; x < nx; ++x)
            {
                Ray myray(x,y,su_right_part,sv_right_part,q,u,v,e);
                Vec3f rayColor;
                
                rayColor = computeColor(myray,scene,scene.max_recursion_depth,false,triangles,vertices,meshes,spheres,materials);
                image[i++] = rayColor.clamp().x; //(int)(rayColor.x*255+0.5);
                image[i++] = rayColor.clamp().y; //(int)(rayColor.y*255+0.5);
                image[i++] = rayColor.clamp().z; //(int)(rayColor.z*255+0.5);
            }
        }
        
        write_ppm(scene.cameras[c].image_name.c_str(), image, nx, ny);
    }

    return 0;

    // CODE ABOVE



}
