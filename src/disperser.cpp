
#include <arrayfire.h>


#define GLM_FORCE_LEFT_HANDED

#include <glm/vec4.hpp> // glm::vec4
#include <glm/mat4x4.hpp> // glm::mat4
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <stdio.h>

#include <vector>

#undef far
#undef near

using namespace af;

void initStep(const array &pos, const array &dir, array &vpos, array &step, array &tdelta, array &tmax)
{
    // x = px < 0 ? int(px) - 1 : int(px);
    // y = py < 0 ? int(py) - 1 : int(py);
    // z = pz < 0 ? int(pz) - 1 : int(pz);

    vpos = floor(pos).as(s32);

    array invdir = 1.0 / dir;

    step = (1 - sign(dir) * 2).as(b8);
    tdelta = abs(invdir);

    tmax = (vpos + (dir >= 0) - pos) * invdir;


    // tMaxX = r.i >= 0 ? (x + 1 - px)*r.ii : (x - px)*r.ii;
    // tMaxY = r.j >= 0 ? (y + 1 - py)*r.ij : (y - py)*r.ij;
    // tMaxZ = r.k >= 0 ? (z + 1 - pz)*r.ik : (z - pz)*r.ik;

    /*
    if (tMaxX < tMaxY) {
        if (tMaxX < tMaxZ) {
            side = 2;
        }
        else {
            side = 1;
        }
    }
    else {
        if (tMaxY < tMaxZ) {
            side = 0;
        }
        else {
            side = 1;
        }
    }
    */
}

void stepOnce(array &vpos, const array &step, const array &tdelta, array &tmax, const array &vid)
{
    array xly = tmax.col(0) < tmax.col(1);
    array xlz = tmax.col(0) < tmax.col(2);
    array ylz = tmax.col(1) < tmax.col(2);

    // Rays are moving if they did not hit anything (for now)
    array moving = vid == 0;

    // Step dimension conditionals
    array cond = join(1,
        (xly && xlz),
        (!xly && ylz),
        ((xly && !xlz) || (!xly && !ylz))
    );

    // Only step moving rays
    cond *= join(1, moving, moving, moving);

    vpos += step * cond;
    tmax += tdelta * cond;

    /*
    public function step() :void {
        if (tMaxX < tMaxY) {
            if (tMaxX < tMaxZ) {
                x += stepX;
                tMaxX += tDeltaX;
                side = 2;
            }
            else {
                z += stepZ;
                tMaxZ += tDeltaZ;
                side = 1;
            }
        }
        else {
            if (tMaxY < tMaxZ) {
                y += stepY;
                tMaxY += tDeltaY;
                side = 0;
            }
            else {
                z += stepZ;
                tMaxZ += tDeltaZ;
                side = 1;
            }
        }
        steps++;
    }
    */

}

static inline int getBlockIndex(const int bx, const int by, const int bz, const int sx, const int sxy)
{
    return bx + by*sx + bz*sxy;
}

static inline void getIndexBlock(const int index, int sx, int sy, int &bx, int &by, int &bz)
{
    bx = index % sx;
    by = (index / sx) % sy;
    bz = index / (sx*sy);
}

template<typename T>
struct Plane {
    T a;
    T b;
    T c;
    T d;

    static glm::vec3 intersectionPoint(const Plane &a, const Plane &b, const Plane &c) {
        glm::tvec3<T> inter{ 0.0, 0.0, 0.0 };

        glm::tvec3<T> bc{
            b.b*c.c - b.c*c.b,
            b.c*c.a - b.a*c.c,
            b.a*c.b - b.b*c.a,
        };

        glm::tvec3<T> ca{
            c.b*a.c - c.c*a.b,
            c.c*a.a - c.a*a.c,
            c.a*a.b - c.b*a.a,
        };

        glm::tvec3<T> ab{
            a.b*b.c - a.c*b.b,
            a.c*b.a - a.a*b.c,
            a.a*b.b - a.b*b.a,
        };

        inter += bc*a.d;
        inter += ca*b.d;
        inter += ab*c.d;

        inter /= a.a*bc.x + a.b*bc.y + a.c*bc.z;

        return inter;

        /*
        public static function intersectionPoint(p:Vector3D, a : Plane, b : Plane, c : Plane) :void {
            var ad : Number = a.d;
            var bd : Number = b.d;
            var cd : Number = c.d;
            var nax : Number = a.a;
            var nay : Number = a.b;
            var naz : Number = a.c;
            var nbx : Number = b.a;
            var nby : Number = b.b;
            var nbz : Number = b.c;
            var ncx : Number = c.a;
            var ncy : Number = c.b;
            var ncz : Number = c.c;

            var px : Number = 0, py : Number = 0, pz : Number = 0;


            var bcx : Number = nby*ncz - nbz*ncy;
            var bcy : Number = nbz*ncx - nbx*ncz;
            var bcz : Number = nbx*ncy - nby*ncx;

            px += ad*bcx;
            py += ad*bcy;
            pz += ad*bcz;

            px += bd*(ncy*naz - ncz*nay);
            py += bd*(ncz*nax - ncx*naz);
            pz += bd*(ncx*nay - ncy*nax);

            px += cd*(nay*nbz - naz*nby);
            py += cd*(naz*nbx - nax*nbz);
            pz += cd*(nax*nby - nay*nbx);

            var scale : Number = 1 / (nax*bcx + nay*bcy + naz*bcz);

            p.x = px*scale;
            p.y = py*scale;
            p.z = pz*scale;
        }
        */
    }

    void fromMatrix(const T* matrix, const T signColA, const int colA, const int colB) {
        /*
        left  .fromMatrix(matrix,  1, 0, 3);
        right .fromMatrix(matrix, -1, 0, 3);
        bottom.fromMatrix(matrix,  1, 1, 3);
        top   .fromMatrix(matrix, -1, 1, 3);
        near  .fromMatrix(matrix,  1, 2, 3);
        far   .fromMatrix(matrix, -1, 2, 3);
        */
        a = signColA * matrix[colA     ] + matrix[colB     ];
        b = signColA * matrix[colA +  4] + matrix[colB +  4];
        c = signColA * matrix[colA +  8] + matrix[colB +  8];
        d = signColA * matrix[colA + 12] + matrix[colB + 12];
        normalize();
    }

    void normalize() {
        T len = sqrtl(a*a + b*b + c*c);
        a /= len;
        b /= len;
        c /= len;
        d /= len;
    }
};

template<typename T>
struct Rect3D {
    glm::tvec4<T> tl;
    glm::tvec4<T> tr;
    glm::tvec4<T> bl;
    glm::tvec4<T> br;

    void fromFrustum(
        const Plane<T> &tla,
        const Plane<T> &tlb,
        const Plane<T> &tlc,

        const Plane<T> &tra,
        const Plane<T> &trb,
        const Plane<T> &trc,

        const Plane<T> &bla,
        const Plane<T> &blb,
        const Plane<T> &blc,

        const Plane<T> &bra,
        const Plane<T> &brb,
        const Plane<T> &brc
    ) {
        tl = Plane<T>::intersectionPoint(tla, tlb, tlc);
        tr = Plane<T>::intersectionPoint(tra, trb, trc);
        bl = Plane<T>::intersectionPoint(bla, blb, blc);
        br = Plane<T>::intersectionPoint(bra, brb, brc);
    }

    glm::tvec4<T> pointOnSurface(T x, T y) {

        glm::tvec4<T> p;

        p = tl*(1 - x)*(1 - y) +
            tr*x*(1 - y) +
            bl*(1 - x)*y +
            br*x*y;
        
        return p;
    }

};

template<typename T>
struct Frustum {

    /*
    Plane<T> left;
    Plane<T> right;
    Plane<T> bottom;
    Plane<T> top;
    Plane<T> near;
    Plane<T> far;
    */

    Rect3D<T> nearRect;
    Rect3D<T> farRect;

    void fromInvMatrix(const glm::mat4 &inv) {


        nearRect.tl = { -1, -1, -1,  1 };
        nearRect.tr = {  1, -1, -1,  1 };
        nearRect.bl = { -1,  1, -1,  1 };
        nearRect.br = {  1,  1, -1,  1 };

        projectCorner(inv, nearRect.tl);
        projectCorner(inv, nearRect.tr);
        projectCorner(inv, nearRect.bl);
        projectCorner(inv, nearRect.br);

        farRect.tl = { -1, -1,  1,  1 };
        farRect.tr = {  1, -1,  1,  1 };
        farRect.bl = { -1,  1,  1,  1 };
        farRect.br = {  1,  1,  1,  1 };

        projectCorner(inv, farRect.tl);
        projectCorner(inv, farRect.tr);
        projectCorner(inv, farRect.bl);
        projectCorner(inv, farRect.br);

    }

private:
    void projectCorner(const glm::mat4 &inv, glm::tvec4<T> &corner) {
        corner = inv*corner;
        corner /= corner.w;
    }


    /*
    void update(const T* matrix) {
        left  .fromMatrix(matrix,  1, 0, 3);
        right .fromMatrix(matrix, -1, 0, 3);
        bottom.fromMatrix(matrix,  1, 1, 3);
        top   .fromMatrix(matrix, -1, 1, 3);
        near  .fromMatrix(matrix,  1, 2, 3);
        far   .fromMatrix(matrix, -1, 2, 3);

        nearRect.fromFrustum(
            near, top, left,
            near, top, right,
            near, bottom, left,
            near, bottom, right
        );

        farRect.fromFrustum(
            far, top, left,
            far, top, right,
            far, bottom, left,
            far, bottom, right
        );
    }
    */



};

int main()
{
    try {
        af::setDevice(1);

        af::info();

        const int width = 256;
        const int height = width;
        const int num = width*height;

        int stepNum = 0;

        array x, y, z, pos, dir;
        array vpos, step, tdelta, tmax;
        array vid;
        array gpos, gindex;

        const int gpower = 4;
        const int grid = 1 << gpower;
        const int gmask = grid - 1;

        std::vector<int> h_ids;
        h_ids.resize(grid*grid*grid);

        std::vector<double> h_pos;
        std::vector<double> h_dir;

        const float center = grid * 0.5;
        const float radius = grid/2 - 1;
        for (size_t iz = 0; iz < grid; iz++) {
            for (size_t iy = 0; iy < grid; iy++) {
                for (size_t ix = 0; ix < grid; ix++) {
                    float fx = ix + 0.5;
                    float fy = iy + 0.5;
                    float fz = iz + 0.5;
                    float dx = fx - center;
                    float dy = fy - center;
                    float dz = fz - center;
                    h_ids[getBlockIndex(ix, iy, iz, grid, grid*grid)] = dx*dx + dy*dy + dz*dz < radius*radius ? 1 : 0;
                    //h_ids[getBlockIndex(ix, iy, iz, grid, grid*grid)] = getBlockIndex(ix, iy, iz, grid, grid*grid);
                    //h_ids[getBlockIndex(ix, iy, iz, grid, grid*grid)] = grid*grid*grid - 1 - getBlockIndex(ix, iy, iz, grid, grid*grid);
                    //h_ids[getBlockIndex(ix, iy, iz, grid, grid*grid)] = 1;
                }
            }
        }

        array ids(grid*grid*grid, &h_ids[0]);
        //array ids(grid, grid, grid, &h_ids[0]);

        //af_print(moddims(ids, grid, grid, grid));

        //getc(stdin); return 0;

        glm::tvec3<double> camPos;
        camPos.x = 0;
        camPos.y = 0;
        camPos.z = 0;

        glm::tvec3<double> camRot;
        camRot.x = 0;
        camRot.y = 0;
        camRot.z = 0;

        glm::mat4 cam;
        Frustum<double> frustum;

        //frustum.update(glm::value_ptr(glm::f64mat4(cam)));


        af::Window window(512, 512, "Test!");
        while (!window.close()) {

            for (stepNum = 0; stepNum < 50; stepNum++) {

                if (stepNum == 0) {


                    //glm::f64mat4 projection = glm::ortho(0.0, (double)width, (double)height, 0.0, 0.1, 1000.0);
                    //glm::f64mat4 projection = glm::orthoLH(0.0, (double)width, (double)height, 0.0, 0.1, 1000.0);
                    //glm::f64mat4 projection = glm::perspectiveFovLH(60. / 180 * glm::pi<double>(), (double)width, (double)height, 0.2, 1000.0);
                    /*
                    glm::f64mat4 projection = glm::perspective(60.,  (double)width / (double)height, 0.1, 1000.0);
                    glm::f64mat4 view{};
                    view = glm::translate(view, camPos);
                    view = glm::rotate(view, camRot.x, glm::f64vec3(0., 0., 1.));
                    view = glm::rotate(view, camRot.y, glm::f64vec3(1., 0., 0.));
                    view = glm::rotate(view, camRot.z, glm::f64vec3(0., 1., 0.));
                    cam = projection * view;
                    glm::mat4 invcam = glm::inverse(cam);

                    frustum.fromInvMatrix(invcam);
                    */

                    glm::f64vec4 p;
                    
                    p = glm::f64vec4(0, 0, 1, 1);

                    //glm::f64vec4 res = projection * p;
                    //res /= res.w;



                    printf("%f %f %f %f\n", res.x, res.y, res.z, res.w);

                    h_pos.resize(width*height * 3);
                    h_dir.resize(width*height * 3);

                    for (size_t iy = 0; iy < height; iy++) {
                        for (size_t ix = 0; ix < width; ix++) {

                            size_t index = ix + iy*width;

                            double fx = (ix + 0.5)/width;
                            double fy = (iy + 0.5)/height;
                            glm::f64vec4 pn = frustum.nearRect.pointOnSurface(fx, fy);
                            glm::f64vec4 pf = frustum.farRect.pointOnSurface(fx, fy);
                        
                            glm::f64vec4 dir = pf - pn;
                            dir = dir / glm::length(dir);

                            h_pos[index + 0 * num] = pn.x;
                            h_pos[index + 1 * num] = pn.y;
                            h_pos[index + 2 * num] = pn.z;
                            h_dir[index + 0 * num] = dir.x;
                            h_dir[index + 1 * num] = dir.y;
                            h_dir[index + 2 * num] = dir.z;
                        }
                    }


                    pos = array(num, 3, &h_pos[0]);
                    dir = array(num, 3, &h_dir[0]);

                    vid = constant(0, num, 1);

                    initStep(pos, dir, vpos, step, tdelta, tmax);

                } else {
                    stepOnce(vpos, step, tdelta, tmax, vid);
                }

                gpos = af::min(af::max(vpos, 0.), (double)gmask);
                //gpos = vpos & gmask;
            
                gindex = array(num, 1);
                gindex = gpos.col(0) + gpos.col(1)*grid + gpos.col(2)*grid*grid;
            
                vid = lookup(ids, gindex, 0);

            }


            window.image(af::reorder(moddims(vid.as(f32), width, height), 1, 0));

            //stepNum++; if (stepNum > 100) stepNum = 0;
        
            window.show();

            getc(stdin);
            //Sleep(20);

            camRot.x += 0.1;

        }

    } catch (af::exception& e) {
        fprintf(stderr, "%s\n", e.what());
        throw;
    }

    //
    //pos = seq(0, width, 1.0/width);

    //pos = join(1, pos, pos);

	return 0;
}