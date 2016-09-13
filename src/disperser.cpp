
#include <arrayfire.h>


#define GLM_FORCE_LEFT_HANDED

#include <glm/vec4.hpp> // glm::vec4
#include <glm/mat4x4.hpp> // glm::mat4
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/quaternion.hpp>

#include <stdio.h>
#include <cstdint>

#include <vector>

#include <mutex>

#undef far
#undef near

enum CoordSide {
    X,
    Y,
    Z
};


#if __cplusplus < 201103L && (!defined(_MSC_VER) || _MSC_VER < 1700)
#error Timer requires C++11
#elif 1
#include <chrono>
class Timer {
    typedef std::chrono::high_resolution_clock clock;
    typedef std::chrono::nanoseconds ns;

    std::mutex mutex;

    clock::time_point start;
    long long counted;

    const char *name;
    bool printed;

public:
    Timer(const char *name)
    {
        this->name = name;
        tick();
    }
    ~Timer()
    {
        print();
    }

    void tick()
    {
        printed = false;
        counted = 0;
        start = clock::now();
    }
    ns tock() const
    {
        return std::chrono::duration_cast<ns>(clock::now() - start);
    }
    long long print()
    {
        long long c = tock().count();
        printed = true;
        const char *unit;
        if (c < 1000) {
            unit = "ns";
        }
        else if (c < 1000000l) {
            c /= 1000l;
            unit = "us";
        }
        else {
            c /= 1000000l;
            unit = "ms";
        }
        printf("%s %*s%lld%s\n", name, (int)(20 - strlen(name)), " ", c, unit);
        return c;
    }
};
#endif

#if 1
#define dtimer(name) Timer timer(name)
#define dtimerInit(var_id, name) Timer timer_ ## var_id {name}
#define dtimerStart(var_id) timer_ ## var_id .tick()
#define dtimerStop(var_id) timer_ ## var_id .count()
#else
#define dtimer(name)
#define dtimerInit(name)
#define dtimerStart(name)
#define dtimerStop(name)
#endif

using namespace af;

static int frame = 0;



static int getBlockIndexXYZ(const int bx, const int by, const int bz, const int grid)
{
    return bx + by*grid + bz*grid*grid;
}

static array getBlockIndexXYZ(const array pos, const int grid)
{
    return pos.col(0) + pos.col(1)*grid + pos.col(2)*grid*grid;
}

template<class T>
static T mortonSplitBy3(T x)
{
    x = x & 0x000003ff;
    x = (x | x << 16) & 0x30000ff;
    x = (x | x << 8)  & 0x0300f00f;
    x = (x | x << 4)  & 0x30c30c3;
    x = (x | x << 2) & 0x9249249;
    return x;
}

static uint32_t getBlockIndexMorton(const int bx, const int by, const int bz, const int grid)
{
    return mortonSplitBy3(bx) | (mortonSplitBy3(by) << 1) | (mortonSplitBy3(bz) << 2);
}

static array getBlockIndexMorton(const array pos, const int grid)
{
    return mortonSplitBy3(pos.col(0)) + (mortonSplitBy3(pos.col(1)) << 1) | (mortonSplitBy3(pos.col(2)) << 2);
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

#define vassert(x, format, ...) if (!(x)) { printf(format, __VA_ARGS__); __debugbreak(); }

static void writeCoords(const float x, const float y, const float z, FILE* file)
{
    fwrite(&x, sizeof(float), 1, file);
    fwrite(&y, sizeof(float), 1, file);
    fwrite(&z, sizeof(float), 1, file);
}

static void writeVector(const std::vector<double>& vec, FILE* file)
{
    fwrite(&vec[0], sizeof(double), vec.size(), file);
    /*
    for (auto num : vec) {
    fwrite(&num, sizeof(double), 1, file);
    }
    //*/
}

template <class T>
static void writeVector(const std::vector<T>& vec, FILE* file)
{
    for (auto num : vec) {
        double n = (double)num;
        fwrite(&n, sizeof(double), 1, file);
    }
}

struct VoxelTrace
{
    // Set on init
    array step, tdelta;

    // Stepped
    array vpos, tmax, side;

    // Looked up value IDs
    array vid;
    array indices;

    array getOrJoin(const array lhs, const array rhs) const
    {
        return lhs.elements() > 0 ? rhs.elements() > 0 ? af::join(0, lhs, rhs) : lhs : rhs;
    }

    VoxelTrace& operator+=(const VoxelTrace &rhs)
    {
        step = getOrJoin(step, rhs.step);
        tdelta = getOrJoin(tdelta, rhs.tdelta);
        vpos = getOrJoin(vpos, rhs.vpos);
        tmax = getOrJoin(tmax, rhs.tmax);
        side = getOrJoin(side, rhs.side);
        vid = getOrJoin(vid, rhs.vid);
        indices = getOrJoin(indices, rhs.indices);
        return *this;
    }

    void filterFrom(const VoxelTrace &src, const array &idx)
    {
        step = af::lookup(src.step, idx, 0);
        tdelta = af::lookup(src.tdelta, idx, 0);
        vpos = af::lookup(src.vpos, idx, 0);
        tmax = af::lookup(src.tmax, idx, 0);
        side = af::lookup(src.side, idx, 0);
        indices = af::lookup(src.indices, idx, 0);
    }

    void reset()
    {
        step = array();
        tdelta = array();
        vpos = array();
        tmax = array();
        side = array();
        vid = array();
        indices = array();
    }

    void initStep(const array &pos, const array &dir)
    {
        vpos = floor(pos).as(s32);

        array invdir = 1.0 / dir;

        step = (sign(dir) * 2).as(u8);
        tdelta = abs(invdir);

        tmax = (vpos + 1 - sign(dir) - pos)*invdir;

        array xly = tmax.col(0) < tmax.col(1);
        array xlz = tmax.col(0) < tmax.col(2);
        array ylz = tmax.col(1) < tmax.col(2);

        side = (((xly & !xlz).as(u8) << 1) | (!xly)*(2 - ylz)).as(u8);

        indices = af::iota(dim4(vpos.dims()[0]));
    }


    void stepOnce()
    {
        if (vpos.elements() <= 0) return;

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

        vpos += (1 - step) * cond;
        tmax += tdelta * cond;
        side = !moving * side + moving*((((xly & !xlz) << 1) | (!xly)*(2 - ylz)).as(u8));
    }

    void lookup(const int grid, const array& ids)
    {
        if (vpos.elements() <= 0) {
            vid = array();
            return;
        }
        const int gmask = grid - 1;
        array gindex = getBlockIndexXYZ(vpos & gmask, grid);
        //gindex = getBlockIndexMorton(gpos, grid);
        vid = af::lookup(ids, gindex, 0);
    }

    void sortResults(VoxelTrace &hits)
    {
        if (vid.elements() <= 0) return;

        array sorted_vid;
        array sorted_vid_index;

        af::sort(sorted_vid, sorted_vid_index, vid, 0, true);

        sorted_vid_index = sorted_vid_index.copy();

        int nonzero = af::count<int>(sorted_vid);

        if (nonzero == 0) return;

        int rownum = sorted_vid.dims()[0];

        // Copy is required before using lookup!
        array hits_indices = sorted_vid_index.rows(rownum - nonzero, rownum - 1).copy();
        hits.vid = sorted_vid.rows(rownum - nonzero, rownum - 1);
        hits.filterFrom(*this, hits_indices);

        if (rownum <= nonzero) {
            reset();
        } else {
            array new_indices = sorted_vid_index.rows(0, rownum - 1 - nonzero).copy();
            vid = sorted_vid.rows(0, rownum - 1 - nonzero);
            filterFrom(*this, new_indices);
        }

    }
};

struct Rays
{
    array screen;
    array pos, dir;
    array t;
};

struct RayBatch
{
    Rays r;
    VoxelTrace v;
    VoxelTrace hits;

    void initStep()
    {
        v.initStep(r.pos, r.dir);
    }

    void stepToTime()
    {
        if (hits.indices.elements() <= 0) {
            r.t = array();
            return;
        }

        array lpos = af::lookup(r.pos, hits.indices, 0);
        array ldir = af::lookup(r.dir, hits.indices, 0);

        array t3 = (hits.vpos - lpos + (ldir < 0)) / ldir;
        r.t = select(hits.side == 1, t3.col(1), t3.col(0));
        r.t = select(hits.side == 2, t3.col(2), r.t);


        //r.t = v.stepToTime(r.pos, r.dir);
        /*
        if (vpos.elements() <= 0) return array();
        vassert(vpos.elements() == pos.elements(), "Mismatched voxel trace and ray sizes");
        array t3 = (vpos - pos + (dir < 0)) / dir;
        array t;
        t = select(side == 1, t3.col(1), t3.col(0));
        t = select(side == 2, t3.col(2), t);
        return t;
        */
    }
};

int main()
{
    try {
        af::setDevice(0);

        af::info();

        const int imageWidth = 512;
        const int imageHeight = imageWidth;
        const int num = imageWidth*imageHeight;

        const char* debugPath = "points.bin";
        FILE* debug = fopen(debugPath, "wb");
        vassert(debug, "Unable to open %s", debugPath);
        
        int step = 0;

        RayBatch b;

        
        /*
            if (tMaxX < tMaxY) {
                if (tMaxX < tMaxZ) {
                    side = 2; // 0
                }
                else {
                    side = 1; // 2
                }
            }
            else {
                if (tMaxY < tMaxZ) {
                    side = 0; // 1
                }
                else {
                    side = 1; // 2
                }
            }
        */

        /*
        double h_tmax[] = {
            1, 2, 3,
            1, 3, 2,
            2, 3, 1,

            2, 1, 3,
            3, 1, 2,
            3, 2, 1
        };

        array tmax(3, 6, h_tmax);
        tmax = af::transpose(tmax);
        af_print(tmax);

        array xly = tmax.col(0) < tmax.col(1);
        array xlz = tmax.col(0) < tmax.col(2);
        array ylz = tmax.col(1) < tmax.col(2);

        af_print(xly);
        af_print(xlz);
        af_print(ylz);

        array side = (((xly & !xlz).as(u8) << 1) | (!xly)*(2 - ylz)).as(u8);

        af_print(side);

        exit(0);
        */



        const int gpower = 4;
        const int grid = 1 << gpower;
        const int gmask = grid - 1;

        std::vector<int> h_ids;
        h_ids.resize(grid*grid*grid);

        std::vector<int> h_screen;
        std::vector<int> h_indices;
        std::vector<double> h_pos;
        std::vector<double> h_dir;
        std::vector<int> h_vpos;
        std::vector<byte> h_step;

        std::vector<float> h_image;
        std::vector<float> h_colors;

        const float center = grid * 0.5;
        const float radius = grid/3 - 1;
        for (size_t iz = 0; iz < grid; iz++) {
            for (size_t iy = 0; iy < grid; iy++) {
                for (size_t ix = 0; ix < grid; ix++) {
                    float fx = ix + 0.5;
                    float fy = iy + 0.5;
                    float fz = iz + 0.5;
                    float dx = fx - center;
                    float dy = fy - center;
                    float dz = fz - center;
                    uint32_t index = getBlockIndexXYZ(ix, iy, iz, grid);
                    //uint32_t index = getBlockIndexMorton(ix, iy, iz, grid);
                    h_ids[index] = dx*dx + dy*dy + dz*dz < radius*radius ? 1 : 0;
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

        glm::f64vec3 camPos{ 0, 0, 0 };
        glm::f64vec3 camDir{ 0, 0, -1 };

        glm::tvec3<double> camRot;
        camRot.x = glm::radians(90.);
        camRot.y = glm::radians(0.);
        camRot.z = glm::radians(0.);

        glm::mat4 cam;
        Frustum<double> frustum;

        std::vector<long long> stepTimes;

        //frustum.update(glm::value_ptr(glm::f64mat4(cam)));

        double fov = 60;

        glm::f64mat4 view{};

        //double t = 0;

        af::Window window(512, 512, "disperser");
        while (!window.close()) {
        //while (frame < 25) {

            printf("frame %d\n", frame);

            //rewind(debug);

            b.hits.reset();

            const int stepNum = 200;
            const int stepHopNum = 20;

            {
                Timer timer("stepping");

                //for (step = 0; step < 100; step++) {
                for (int stepHop = 0; stepHop < stepHopNum; stepHop++, step++) {

                    if (step == 0) {

                        //dtimer("initStep");

                        //glm::f64mat4 projection = glm::ortho(0.0, (double)imageWidth, (double)imageHeight, 0.0, 0.1, 1000.0);
                        //glm::f64mat4 projection = glm::orthoLH(0.0, (double)imageWidth, (double)imageHeight, 0.0, 0.1, 1000.0);
                        //glm::f64mat4 projection = glm::perspectiveFovLH(60. / 180 * glm::pi<double>(), (double)imageWidth, (double)imageHeight, 0.2, 1000.0);
                        /*
                        glm::f64mat4 projection = glm::perspective(60.,  (double)imageWidth / (double)imageHeight, 0.1, 1000.0);
                        glm::f64mat4 view{};
                        view = glm::translate(view, camPos);
                        view = glm::rotate(view, camRot.x, glm::f64vec3(0., 0., 1.));
                        view = glm::rotate(view, camRot.y, glm::f64vec3(1., 0., 0.));
                        view = glm::rotate(view, camRot.z, glm::f64vec3(0., 1., 0.));
                        cam = projection * view;
                        glm::mat4 invcam = glm::inverse(cam);

                        frustum.fromInvMatrix(invcam);
                        */

                        double imageAspect = (double)imageWidth / imageHeight;
                        double fovTan = tan(glm::radians(fov) / 2);

                        //view = glm::lookAt(camPos, camPos + camDir, glm::f64vec3(0, 0, 1));

                        glm::quat qrot = glm::quat(camRot);

                        glm::f64mat4 view{};
                        view = glm::toMat4(qrot);
                        view = glm::translate(view, camPos);


                        /*
                        printf("0: %f %f %f \n", view[0][0], view[1][0], view[2][0]);
                        printf("1: %f %f %f \n", view[0][1], view[1][1], view[2][1]);
                        printf("2: %f %f %f \n", view[0][2], view[1][2], view[2][2]);
                        printf("3: %f %f %f \n", view[0][3], view[1][3], view[2][3]);

                        //getc(stdin);

                        glm::f64vec4 p{ 0, 1, 0, 0 };

                        p = view * p;

                        printf("%f %f %f %f \n", p.x, p.y, p.z, p.w);


                        view = glm::inverse(view);
                        */
                        //view = glm::rotate(view, camRot.z, glm::f64vec3(0., 0., 1.));
                        //view = glm::rotate(view, camRot.y, glm::f64vec3(0., 1., 0.));
                        //view = glm::rotate(view, camRot.x, glm::f64vec3(1., 0., 0.));
                        //view = glm::eulerAngleXY(camRot.y, camRot.x);
                        //view = glm::translate(view, camPos);

                        //glm::f64vec4 origin = view * glm::f64vec4(0, 0, 0, 1);
                        //glm::f64vec4 direction = view * glm::f64vec4(px, py, -1, 0);
                        //direction = glm::normalize(direction);


                        //glm::f64vec3 origin{ 0, 0, 0 };
                        //glm::f64vec3 direction = glm::f64vec3(px, py, -1) - origin;
                        //direction = glm::normalize(direction);

                        //printf("ori %f %f %f %f\n", origin.x, origin.y, origin.z, origin.w);
                        //printf("dir %f %f %f %f\n", direction.x, direction.y, direction.z, direction.w);

                        //getc(stdin); exit(1);

                        h_screen.resize(num * 2);
                        h_pos.resize(imageWidth*imageHeight * 3);
                        h_dir.resize(imageWidth*imageHeight * 3);
                        h_vpos.resize(imageWidth*imageHeight * 3);
                        h_step.resize(imageWidth*imageHeight * 3);

                        for (size_t iy = 0; iy < imageHeight; iy++) {
                            for (size_t ix = 0; ix < imageWidth; ix++) {

                                size_t index = ix + iy*imageWidth;

                                /*
                                double fx = (ix + 0.5)/imageWidth;
                                double fy = (iy + 0.5)/imageHeight;
                                glm::f64vec4 pn = frustum.nearRect.pointOnSurface(fx, fy);
                                glm::f64vec4 pf = frustum.farRect.pointOnSurface(fx, fy);

                                glm::f64vec4 dir = pf - pn;
                                dir = dir / glm::length(dir);
                                */

                                //fovTan = imageAspect = 1;
                                double px = (2 * (ix + 0.5) / imageWidth - 1) * fovTan * imageAspect;
                                double py = (1 - 2 * (iy + 0.5) / imageHeight) * fovTan;

                                glm::f64vec4 o = view * glm::f64vec4(0, 0, 0, 1);
                                o /= o.w;

                                //if (ix == 0 && iy == 0) {
                                    //printf("%f %f \n", px, py);
                                    //getc(stdin);
                                //}

                                glm::f64vec4 d = glm::f64vec4(px, py, -1, 0);
                                d = view * d;
                                d = glm::normalize(d);

                                /*
                                double len = sqrt(d.x*d.x + d.y*d.y + d.z*d.z);
                                d.x /= len;
                                d.y /= len;
                                d.z /= len;
                                */

                                //glm::f64vec4 dir = glm::f64vec4(cos(t*0.1)*0.0, cos(t*0.02)*1.2, 0, 0);

                                h_screen[index + 0 * num] = ix;
                                h_screen[index + 1 * num] = iy;
                                h_pos[index + 0 * num] = o.x;
                                h_pos[index + 1 * num] = o.y;
                                h_pos[index + 2 * num] = o.z;
                                h_dir[index + 0 * num] = d.x;
                                h_dir[index + 1 * num] = d.y;
                                h_dir[index + 2 * num] = d.z;
                            }
                        }

                        //h_pos[0 + 0 * num] = h_pos[0 + 1 * num] = h_pos[0 + 2 * num] = -10;
                        //h_pos[1 + 0 * num] = h_pos[1 + 1 * num] = h_pos[1 + 2 * num] = 10;

                        //h_dir[0 + 0 * num] = h_dir[0 + 1 * num] = h_dir[0 + 2 * num] = 0;
                        //h_dir[1 + 0 * num] = h_dir[1 + 1 * num] = h_dir[1 + 2 * num] = 0;

                        //writeVector(h_dir, debug);

                        b.r.screen = array(num, 2, &h_screen[0]);
                        b.r.pos = array(num, 3, &h_pos[0]);
                        b.r.dir = array(num, 3, &h_dir[0]);
                        b.r.t = constant(0., 3, f64);

                        //printf("%d", h_dir.size());

                        b.initStep();
                    }
                    else {
                        //dtimer("stepOnce");

                        b.v.stepOnce();

                        //stepOnce(vpos, step, tdelta, tmax, side, vid);
                        //initStep(pos, dir, vpos, step, tdelta, tmax);
                    }

                    //gpos = af::min(af::max(vpos, 0.), (double)gmask);

                    b.v.lookup(grid, ids);

                    VoxelTrace hits;
                    b.v.sortResults(hits);
                    b.hits += hits;
                    //printf("%d   %d hits\n", step, b.hits.vid.elements());
                }

                if (stepTimes.size() >= 5) stepTimes.erase(stepTimes.begin());
                stepTimes.push_back(timer.print());
                long long avg = 0;
                for (auto &time : stepTimes) {
                    avg += time;
                }
                avg /= stepTimes.size();
                printf("avg step: %lld ms\n", avg);
            }

            //printf("%d", vpos.dims(0));

            

            //pos.host(&h_pos[0]);

            //af::allocHost()
/*
            pos.host(&h_pos[0]);
            writeVector(h_pos, debug);
*/
/*
            vpos.host(&h_vpos[0]);
            writeVector(h_vpos, debug);
*/
/*
            step.host(&h_step[0]);
            writeVector(h_step, debug);
*/

            
            //pos += dir;

            {
                dtimer("step to time");
                b.stepToTime();
            }

            {

            }

            
            // Render side as color
            //array color = vid*((side + 1) / 3.0);

            // Render depth as color
            //array color = 1 - b.r.t*0.01;

            array image;
            {
                dtimer("color");

                h_image.resize(num);
                if (step == stepHopNum) {
                //if ((step/stepHopNum)%3 == 0) {
                    // Reset image
                    memset(&h_image[0], 0, num * sizeof(h_image[0]));
                }

                if (b.r.t.elements() > 0) {
                    array colors = (1 - b.r.t*0.005*((b.hits.side + 1) / 3.0))*b.hits.vid;
                    array screen = af::lookup(b.r.screen, b.hits.indices, 0);

                    h_colors.resize(colors.elements());
                    h_screen.resize(screen.elements());
                    //h_indices.resize(b.hits.indices.elements());

                    colors.as(f32).host(&h_colors[0]);
                    screen.as(s32).host(&h_screen[0]);
                    //b.hits.indices.as(s32).host(&h_indices[0]);

                    //af_print(screen);
                    int num_screen = screen.dims()[0];
                    for (int i = 0; i < num_screen; i++) {
                        int x = h_screen[i];
                        int y = h_screen[i + num_screen];
                        //h_image[x + y*imageWidth] = (float)h_indices[x + y*imageWidth] / (imageWidth*imageHeight);
                        //h_image[x + y*imageWidth] = 1 - (float)(step + (float)i / num_screen) / 150;
                        //h_image[x + y*imageWidth] = 1 - (float)(step + (float)i / num_screen) / 150 - h_colors[i];
                        h_image[x + y*imageWidth] = h_colors[i];
                    }
                    //image = array(imageWidth, imageHeight, &h_colors[0]);
                    //image = af::reorder(moddims(colors.as(f32), imageWidth, imageHeight), 1, 0);
                }

                image = array(imageWidth, imageHeight, &h_image[0]);

                //color = gindex*0.0001;
            }

            {
                dtimer("image");
                if (image.elements() > 0) window.image(image);
                //if (color.elements() > 0) window.image(af::reorder(moddims(color.as(f32), imageWidth, imageHeight), 1, 0));
            }
            //window.image(af::reorder(moddims(vid.as(f32), imageWidth, imageHeight), 1, 0));

            //window.scatter3(pos.as(f32));
            //window.scatter3(dir.as(f32));
            //window.plot3(dir.as(f32));
            
            window.show();

            step++; if (step > stepNum) step = 0;
            
            if (step == 0) {
                frame++;
                camPos.z -= 2;
                camRot.x += 0.04;
                camRot.z += 0.05;
            }

            //getc(stdin);
            //break;

            //Sleep(16);

            //camPos.x = sin(t*0.04) * 10;

            //printf("%f \n", camPos.x);

            //double a = t*0.05;
            //double r = 2;

            //fov = (cos(a) + 1) / 2 * 180;

            //camDir.x = cos(a) * r;
            //camDir.y = sin(a) * r;

            //printf("%f %f %f \n", camDir.x, camDir.y, camDir.z);
            
            //camRot.x = glm::radians(0 + cos(a) * 20);
            //camRot.y = glm::radians(0.);

            //camRot.x += 0.005;
            //camRot.y += 0.002;
            //camRot.z += 0.01;

            //camPos.z -= 1;

            //break;

            //Sleep(1000);

        }

        fclose(debug);

    } catch (af::exception& e) {
        fprintf(stderr, "%s\n", e.what());
        throw;
    }

    //
    //pos = seq(0, width, 1.0/width);

    //pos = join(1, pos, pos);

    return 0;
}