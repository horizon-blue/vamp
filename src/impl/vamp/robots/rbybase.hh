#pragma once

#include <vamp/vector.hh>
#include <vamp/vector/math.hh>
#include <vamp/collision/environment.hh>
#include <vamp/collision/validity.hh>

// NOLINTBEGIN(*-magic-numbers)
namespace vamp::robots
{
struct Rbybase
{
    static constexpr char* name = "rbybase";
    static constexpr std::size_t dimension = 3;
    static constexpr std::size_t n_spheres = 24;
    static constexpr float min_radius = 0.029999999329447746;
    static constexpr float max_radius = 0.10000000149011612;
    static constexpr std::size_t resolution = 32;

    static constexpr std::array<std::string_view, dimension> joint_names = {"x", "y", "theta"};
    static constexpr char* end_effector = "base";

    using Configuration = FloatVector<dimension>;
    using ConfigurationArray = std::array<FloatT, dimension>;

    struct alignas(FloatVectorAlignment) ConfigurationBuffer
        : std::array<float, Configuration::num_scalars_rounded>
    {
    };

    template <std::size_t rake>
    using ConfigurationBlock = FloatVector<rake, dimension>;

    template <std::size_t rake>
    struct Spheres
    {
        FloatVector<rake, n_spheres> x;
        FloatVector<rake, n_spheres> y;
        FloatVector<rake, n_spheres> z;
        FloatVector<rake, n_spheres> r;
    };

    alignas(Configuration::S::Alignment) static constexpr std::array<float, dimension> s_m{
        5.0, 5.0, 6.283180236816406
    };

    alignas(Configuration::S::Alignment) static constexpr std::array<float, dimension> s_a{
        -2.5, -2.5, -3.141590118408203
    };

    alignas(Configuration::S::Alignment) static constexpr std::array<float, dimension> d_m{
        0.20000000298023224, 0.20000000298023224, 0.15915507078170776
    };

    static inline void scale_configuration(Configuration& q) noexcept
    {
        q = q * Configuration(s_m) + Configuration(s_a);
    }

    static inline void descale_configuration(Configuration& q) noexcept
    {
        q = (q - Configuration(s_a)) * Configuration(d_m);
    }

    template <std::size_t rake>
    static inline void scale_configuration_block(ConfigurationBlock<rake> &q) noexcept
    {
        q[0] = -2.5 + (q[0] * 5.0);
q[1] = -2.5 + (q[1] * 5.0);
q[2] = -3.141590118408203 + (q[2] * 6.283180236816406);

    }

    template <std::size_t rake>
    static inline void descale_configuration_block(ConfigurationBlock<rake> & q) noexcept
    {
        q[0] = 0.20000000298023224 * (q[0] - -2.5);
q[1] = 0.20000000298023224 * (q[1] - -2.5);
q[2] = 0.15915507078170776 * (q[2] - -3.141590118408203);

    }

    inline static auto space_measure() noexcept -> float
    {
        return 157.0795;
    }

    template <std::size_t rake>
    static inline void sphere_fk(const ConfigurationBlock<rake> &x, Spheres<rake> &out) noexcept
    {
        std::array<FloatVector<rake, 1>, 4> v;
        std::array<FloatVector<rake, 1>, 96> y;

           v[0] = cos(x[2]);
   v[1] = sin(x[2]);
   v[2] = - v[1];
   v[3] = -0.228 + x[0];
   y[0] = 0.23 * v[0] + 0.175 * v[2] + v[3];
   y[1] = 0.23 * v[1] + 0.175 * v[0] + x[1];
   y[4] = 0.23 * v[0] + 0.075 * v[2] + v[3];
   y[5] = 0.23 * v[1] + 0.075 * v[0] + x[1];
   y[8] = 0.23 * v[0] + v[3];
   y[9] = 0.23 * v[1] + x[1];
   y[12] = 0.23 * v[0] + -0.075 * v[2] + v[3];
   y[13] = 0.23 * v[1] + -0.075 * v[0] + x[1];
   y[16] = 0.23 * v[0] + -0.175 * v[2] + v[3];
   y[17] = 0.23 * v[1] + -0.175 * v[0] + x[1];
   y[20] = 0.225 * v[0] + 0.1725 * v[2] + v[3];
   y[21] = 0.225 * v[1] + 0.1725 * v[0] + x[1];
   y[24] = 0.225 * v[0] + -0.1725 * v[2] + v[3];
   y[25] = 0.225 * v[1] + -0.1725 * v[0] + x[1];
   y[28] = 0.12 * v[0] + 0.175 * v[2] + v[3];
   y[29] = 0.12 * v[1] + 0.175 * v[0] + x[1];
   y[32] = 0.12 * v[0] + -0.175 * v[2] + v[3];
   y[33] = 0.12 * v[1] + -0.175 * v[0] + x[1];
   y[36] = 0.01 * v[0] + 0.175 * v[2] + v[3];
   y[37] = 0.01 * v[1] + 0.175 * v[0] + x[1];
   y[40] = 0.01 * v[0] + -0.175 * v[2] + v[3];
   y[41] = 0.01 * v[1] + -0.175 * v[0] + x[1];
   y[44] = -0.1 * v[0] + 0.175 * v[2] + v[3];
   y[45] = -0.1 * v[1] + 0.175 * v[0] + x[1];
   y[48] = -0.1 * v[0] + -0.175 * v[2] + v[3];
   y[49] = -0.1 * v[1] + -0.175 * v[0] + x[1];
   y[52] = -0.2 * v[0] + 0.15 * v[2] + v[3];
   y[53] = -0.2 * v[1] + 0.15 * v[0] + x[1];
   y[56] = -0.2 * v[0] + -0.15 * v[2] + v[3];
   y[57] = -0.2 * v[1] + -0.15 * v[0] + x[1];
   y[60] = -0.28 * v[0] + 0.12 * v[2] + v[3];
   y[61] = -0.28 * v[1] + 0.12 * v[0] + x[1];
   y[64] = -0.28 * v[0] + 0.04 * v[2] + v[3];
   y[65] = -0.28 * v[1] + 0.04 * v[0] + x[1];
   y[68] = -0.28 * v[0] + -0.04 * v[2] + v[3];
   y[69] = -0.28 * v[1] + -0.04 * v[0] + x[1];
   y[72] = -0.28 * v[0] + -0.12 * v[2] + v[3];
   y[73] = -0.28 * v[1] + -0.12 * v[0] + x[1];
   y[76] = -0.26 * v[0] + 0.085 * v[2] + v[3];
   y[77] = -0.26 * v[1] + 0.085 * v[0] + x[1];
   y[80] = -0.26 * v[0] + -0.085 * v[2] + v[3];
   y[81] = -0.26 * v[1] + -0.085 * v[0] + x[1];
   y[84] = -0.25 * v[0] + v[3];
   y[85] = -0.25 * v[1] + x[1];
   y[88] = 0.228 * v[0] + 0.265 * v[2] + v[3];
   y[89] = 0.228 * v[1] + 0.265 * v[0] + x[1];
   y[92] = 0.228 * v[0] + -0.265 * v[2] + v[3];
   y[93] = 0.228 * v[1] + -0.265 * v[0] + x[1];
   // dependent variables without operations
   y[2] = 0.13;
   y[3] = 0.100000001490116;
   y[6] = 0.125;
   y[7] = 0.100000001490116;
   y[10] = 0.125;
   y[11] = 0.100000001490116;
   y[14] = 0.125;
   y[15] = 0.100000001490116;
   y[18] = 0.13;
   y[19] = 0.100000001490116;
   y[22] = 0.24;
   y[23] = 0.0299999993294477;
   y[26] = 0.24;
   y[27] = 0.0299999993294477;
   y[30] = 0.11;
   y[31] = 0.0799999982118607;
   y[34] = 0.11;
   y[35] = 0.0799999982118607;
   y[38] = 0.11;
   y[39] = 0.0799999982118607;
   y[42] = 0.11;
   y[43] = 0.0799999982118607;
   y[46] = 0.11;
   y[47] = 0.0799999982118607;
   y[50] = 0.11;
   y[51] = 0.0799999982118607;
   y[54] = 0.11;
   y[55] = 0.0799999982118607;
   y[58] = 0.11;
   y[59] = 0.0799999982118607;
   y[62] = 0.11;
   y[63] = 0.0799999982118607;
   y[66] = 0.11;
   y[67] = 0.0799999982118607;
   y[70] = 0.11;
   y[71] = 0.0799999982118607;
   y[74] = 0.11;
   y[75] = 0.0799999982118607;
   y[78] = 0.05;
   y[79] = 0.0500000007450581;
   y[82] = 0.05;
   y[83] = 0.0500000007450581;
   y[86] = 0.275;
   y[87] = 0.100000001490116;
   y[90] = 0.1;
   y[91] = 0.100000001490116;
   y[94] = 0.1;
   y[95] = 0.100000001490116;


        for (auto i = 0U; i < 24; ++i)
        {
            out.x[i] = y[i * 4 + 0];
            out.y[i] = y[i * 4 + 1];
            out.z[i] = y[i * 4 + 2];
            out.r[i] = y[i * 4 + 3];
        }
    }

    using Debug = std::pair<std::vector<std::vector<std::string>>, std::vector<std::pair<std::size_t, std::size_t>>>;

    template <std::size_t rake>
        static inline auto fkcc_debug(
            const vamp::collision::Environment<FloatVector<rake>> &environment,
            const ConfigurationBlock<rake> &x) noexcept -> Debug
    {
        std::array<FloatVector<rake, 1>, 4> v;
        std::array<FloatVector<rake, 1>, 108> y;

           v[0] = cos(x[2]);
   v[1] = sin(x[2]);
   v[2] = - v[1];
   v[3] = -0.228 + x[0];
   y[0] = 0.23 * v[0] + 0.175 * v[2] + v[3];
   y[1] = 0.23 * v[1] + 0.175 * v[0] + x[1];
   y[4] = 0.23 * v[0] + 0.075 * v[2] + v[3];
   y[5] = 0.23 * v[1] + 0.075 * v[0] + x[1];
   y[8] = 0.23 * v[0] + v[3];
   y[9] = 0.23 * v[1] + x[1];
   y[12] = 0.23 * v[0] + -0.075 * v[2] + v[3];
   y[13] = 0.23 * v[1] + -0.075 * v[0] + x[1];
   y[16] = 0.23 * v[0] + -0.175 * v[2] + v[3];
   y[17] = 0.23 * v[1] + -0.175 * v[0] + x[1];
   y[20] = 0.225 * v[0] + 0.1725 * v[2] + v[3];
   y[21] = 0.225 * v[1] + 0.1725 * v[0] + x[1];
   y[24] = 0.225 * v[0] + -0.1725 * v[2] + v[3];
   y[25] = 0.225 * v[1] + -0.1725 * v[0] + x[1];
   y[28] = 0.12 * v[0] + 0.175 * v[2] + v[3];
   y[29] = 0.12 * v[1] + 0.175 * v[0] + x[1];
   y[32] = 0.12 * v[0] + -0.175 * v[2] + v[3];
   y[33] = 0.12 * v[1] + -0.175 * v[0] + x[1];
   y[36] = 0.01 * v[0] + 0.175 * v[2] + v[3];
   y[37] = 0.01 * v[1] + 0.175 * v[0] + x[1];
   y[40] = 0.01 * v[0] + -0.175 * v[2] + v[3];
   y[41] = 0.01 * v[1] + -0.175 * v[0] + x[1];
   y[44] = -0.1 * v[0] + 0.175 * v[2] + v[3];
   y[45] = -0.1 * v[1] + 0.175 * v[0] + x[1];
   y[48] = -0.1 * v[0] + -0.175 * v[2] + v[3];
   y[49] = -0.1 * v[1] + -0.175 * v[0] + x[1];
   y[52] = -0.2 * v[0] + 0.15 * v[2] + v[3];
   y[53] = -0.2 * v[1] + 0.15 * v[0] + x[1];
   y[56] = -0.2 * v[0] + -0.15 * v[2] + v[3];
   y[57] = -0.2 * v[1] + -0.15 * v[0] + x[1];
   y[60] = -0.28 * v[0] + 0.12 * v[2] + v[3];
   y[61] = -0.28 * v[1] + 0.12 * v[0] + x[1];
   y[64] = -0.28 * v[0] + 0.04 * v[2] + v[3];
   y[65] = -0.28 * v[1] + 0.04 * v[0] + x[1];
   y[68] = -0.28 * v[0] + -0.04 * v[2] + v[3];
   y[69] = -0.28 * v[1] + -0.04 * v[0] + x[1];
   y[72] = -0.28 * v[0] + -0.12 * v[2] + v[3];
   y[73] = -0.28 * v[1] + -0.12 * v[0] + x[1];
   y[76] = -0.26 * v[0] + 0.085 * v[2] + v[3];
   y[77] = -0.26 * v[1] + 0.085 * v[0] + x[1];
   y[80] = -0.26 * v[0] + -0.085 * v[2] + v[3];
   y[81] = -0.26 * v[1] + -0.085 * v[0] + x[1];
   y[84] = -0.25 * v[0] + v[3];
   y[85] = -0.25 * v[1] + x[1];
   y[88] = 0.228 * v[0] + 0.265 * v[2] + v[3];
   y[89] = 0.228 * v[1] + 0.265 * v[0] + x[1];
   y[92] = 0.228 * v[0] + -0.265 * v[2] + v[3];
   y[93] = 0.228 * v[1] + -0.265 * v[0] + x[1];
   y[96] = 0.0019322152948007 * v[0] + -2.02185565097713e-18 * v[2] + v[3];
   y[97] = 0.0019322152948007 * v[1] + -2.02185565097713e-18 * v[0] + x[1];
   y[100] = 0.228000000119209 * v[0] + 0.264999985694885 * v[2] + v[3];
   y[101] = 0.228000000119209 * v[1] + 0.264999985694885 * v[0] + x[1];
   y[104] = 0.228000000119209 * v[0] + -0.264999985694885 * v[2] + v[3];
   y[105] = 0.228000000119209 * v[1] + -0.264999985694885 * v[0] + x[1];
   // dependent variables without operations
   y[2] = 0.13;
   y[3] = 0.100000001490116;
   y[6] = 0.125;
   y[7] = 0.100000001490116;
   y[10] = 0.125;
   y[11] = 0.100000001490116;
   y[14] = 0.125;
   y[15] = 0.100000001490116;
   y[18] = 0.13;
   y[19] = 0.100000001490116;
   y[22] = 0.24;
   y[23] = 0.0299999993294477;
   y[26] = 0.24;
   y[27] = 0.0299999993294477;
   y[30] = 0.11;
   y[31] = 0.0799999982118607;
   y[34] = 0.11;
   y[35] = 0.0799999982118607;
   y[38] = 0.11;
   y[39] = 0.0799999982118607;
   y[42] = 0.11;
   y[43] = 0.0799999982118607;
   y[46] = 0.11;
   y[47] = 0.0799999982118607;
   y[50] = 0.11;
   y[51] = 0.0799999982118607;
   y[54] = 0.11;
   y[55] = 0.0799999982118607;
   y[58] = 0.11;
   y[59] = 0.0799999982118607;
   y[62] = 0.11;
   y[63] = 0.0799999982118607;
   y[66] = 0.11;
   y[67] = 0.0799999982118607;
   y[70] = 0.11;
   y[71] = 0.0799999982118607;
   y[74] = 0.11;
   y[75] = 0.0799999982118607;
   y[78] = 0.05;
   y[79] = 0.0500000007450581;
   y[82] = 0.05;
   y[83] = 0.0500000007450581;
   y[86] = 0.275;
   y[87] = 0.100000001490116;
   y[90] = 0.1;
   y[91] = 0.100000001490116;
   y[94] = 0.1;
   y[95] = 0.100000001490116;
   y[98] = 0.136396303772926;
   y[99] = 0.387542754411697;
   y[102] = 0.100000001490116;
   y[103] = 0.100000001490116;
   y[106] = 0.100000001490116;
   y[107] = 0.100000001490116;


        Debug output;

        
        output.first.emplace_back(
            sphere_environment_get_collisions<decltype(x[0])>(
                environment,
                y[0],
                y[1],
                y[2],
                y[3]));
        
        output.first.emplace_back(
            sphere_environment_get_collisions<decltype(x[0])>(
                environment,
                y[4],
                y[5],
                y[6],
                y[7]));
        
        output.first.emplace_back(
            sphere_environment_get_collisions<decltype(x[0])>(
                environment,
                y[8],
                y[9],
                y[10],
                y[11]));
        
        output.first.emplace_back(
            sphere_environment_get_collisions<decltype(x[0])>(
                environment,
                y[12],
                y[13],
                y[14],
                y[15]));
        
        output.first.emplace_back(
            sphere_environment_get_collisions<decltype(x[0])>(
                environment,
                y[16],
                y[17],
                y[18],
                y[19]));
        
        output.first.emplace_back(
            sphere_environment_get_collisions<decltype(x[0])>(
                environment,
                y[20],
                y[21],
                y[22],
                y[23]));
        
        output.first.emplace_back(
            sphere_environment_get_collisions<decltype(x[0])>(
                environment,
                y[24],
                y[25],
                y[26],
                y[27]));
        
        output.first.emplace_back(
            sphere_environment_get_collisions<decltype(x[0])>(
                environment,
                y[28],
                y[29],
                y[30],
                y[31]));
        
        output.first.emplace_back(
            sphere_environment_get_collisions<decltype(x[0])>(
                environment,
                y[32],
                y[33],
                y[34],
                y[35]));
        
        output.first.emplace_back(
            sphere_environment_get_collisions<decltype(x[0])>(
                environment,
                y[36],
                y[37],
                y[38],
                y[39]));
        
        output.first.emplace_back(
            sphere_environment_get_collisions<decltype(x[0])>(
                environment,
                y[40],
                y[41],
                y[42],
                y[43]));
        
        output.first.emplace_back(
            sphere_environment_get_collisions<decltype(x[0])>(
                environment,
                y[44],
                y[45],
                y[46],
                y[47]));
        
        output.first.emplace_back(
            sphere_environment_get_collisions<decltype(x[0])>(
                environment,
                y[48],
                y[49],
                y[50],
                y[51]));
        
        output.first.emplace_back(
            sphere_environment_get_collisions<decltype(x[0])>(
                environment,
                y[52],
                y[53],
                y[54],
                y[55]));
        
        output.first.emplace_back(
            sphere_environment_get_collisions<decltype(x[0])>(
                environment,
                y[56],
                y[57],
                y[58],
                y[59]));
        
        output.first.emplace_back(
            sphere_environment_get_collisions<decltype(x[0])>(
                environment,
                y[60],
                y[61],
                y[62],
                y[63]));
        
        output.first.emplace_back(
            sphere_environment_get_collisions<decltype(x[0])>(
                environment,
                y[64],
                y[65],
                y[66],
                y[67]));
        
        output.first.emplace_back(
            sphere_environment_get_collisions<decltype(x[0])>(
                environment,
                y[68],
                y[69],
                y[70],
                y[71]));
        
        output.first.emplace_back(
            sphere_environment_get_collisions<decltype(x[0])>(
                environment,
                y[72],
                y[73],
                y[74],
                y[75]));
        
        output.first.emplace_back(
            sphere_environment_get_collisions<decltype(x[0])>(
                environment,
                y[76],
                y[77],
                y[78],
                y[79]));
        
        output.first.emplace_back(
            sphere_environment_get_collisions<decltype(x[0])>(
                environment,
                y[80],
                y[81],
                y[82],
                y[83]));
        
        output.first.emplace_back(
            sphere_environment_get_collisions<decltype(x[0])>(
                environment,
                y[84],
                y[85],
                y[86],
                y[87]));
        
        output.first.emplace_back(
            sphere_environment_get_collisions<decltype(x[0])>(
                environment,
                y[88],
                y[89],
                y[90],
                y[91]));
        
        output.first.emplace_back(
            sphere_environment_get_collisions<decltype(x[0])>(
                environment,
                y[92],
                y[93],
                y[94],
                y[95]));
        

        

        return output;
    }

    template <std::size_t rake>
        static inline bool fkcc(
            const vamp::collision::Environment<FloatVector<rake>> &environment,
            const ConfigurationBlock<rake> &x) noexcept
    {
        std::array<FloatVector<rake, 1>, 4> v;
        std::array<FloatVector<rake, 1>, 108> y;

           v[0] = cos(x[2]);
   v[1] = sin(x[2]);
   v[2] = - v[1];
   v[3] = -0.228 + x[0];
   y[0] = 0.23 * v[0] + 0.175 * v[2] + v[3];
   y[1] = 0.23 * v[1] + 0.175 * v[0] + x[1];
   y[4] = 0.23 * v[0] + 0.075 * v[2] + v[3];
   y[5] = 0.23 * v[1] + 0.075 * v[0] + x[1];
   y[8] = 0.23 * v[0] + v[3];
   y[9] = 0.23 * v[1] + x[1];
   y[12] = 0.23 * v[0] + -0.075 * v[2] + v[3];
   y[13] = 0.23 * v[1] + -0.075 * v[0] + x[1];
   y[16] = 0.23 * v[0] + -0.175 * v[2] + v[3];
   y[17] = 0.23 * v[1] + -0.175 * v[0] + x[1];
   y[20] = 0.225 * v[0] + 0.1725 * v[2] + v[3];
   y[21] = 0.225 * v[1] + 0.1725 * v[0] + x[1];
   y[24] = 0.225 * v[0] + -0.1725 * v[2] + v[3];
   y[25] = 0.225 * v[1] + -0.1725 * v[0] + x[1];
   y[28] = 0.12 * v[0] + 0.175 * v[2] + v[3];
   y[29] = 0.12 * v[1] + 0.175 * v[0] + x[1];
   y[32] = 0.12 * v[0] + -0.175 * v[2] + v[3];
   y[33] = 0.12 * v[1] + -0.175 * v[0] + x[1];
   y[36] = 0.01 * v[0] + 0.175 * v[2] + v[3];
   y[37] = 0.01 * v[1] + 0.175 * v[0] + x[1];
   y[40] = 0.01 * v[0] + -0.175 * v[2] + v[3];
   y[41] = 0.01 * v[1] + -0.175 * v[0] + x[1];
   y[44] = -0.1 * v[0] + 0.175 * v[2] + v[3];
   y[45] = -0.1 * v[1] + 0.175 * v[0] + x[1];
   y[48] = -0.1 * v[0] + -0.175 * v[2] + v[3];
   y[49] = -0.1 * v[1] + -0.175 * v[0] + x[1];
   y[52] = -0.2 * v[0] + 0.15 * v[2] + v[3];
   y[53] = -0.2 * v[1] + 0.15 * v[0] + x[1];
   y[56] = -0.2 * v[0] + -0.15 * v[2] + v[3];
   y[57] = -0.2 * v[1] + -0.15 * v[0] + x[1];
   y[60] = -0.28 * v[0] + 0.12 * v[2] + v[3];
   y[61] = -0.28 * v[1] + 0.12 * v[0] + x[1];
   y[64] = -0.28 * v[0] + 0.04 * v[2] + v[3];
   y[65] = -0.28 * v[1] + 0.04 * v[0] + x[1];
   y[68] = -0.28 * v[0] + -0.04 * v[2] + v[3];
   y[69] = -0.28 * v[1] + -0.04 * v[0] + x[1];
   y[72] = -0.28 * v[0] + -0.12 * v[2] + v[3];
   y[73] = -0.28 * v[1] + -0.12 * v[0] + x[1];
   y[76] = -0.26 * v[0] + 0.085 * v[2] + v[3];
   y[77] = -0.26 * v[1] + 0.085 * v[0] + x[1];
   y[80] = -0.26 * v[0] + -0.085 * v[2] + v[3];
   y[81] = -0.26 * v[1] + -0.085 * v[0] + x[1];
   y[84] = -0.25 * v[0] + v[3];
   y[85] = -0.25 * v[1] + x[1];
   y[88] = 0.228 * v[0] + 0.265 * v[2] + v[3];
   y[89] = 0.228 * v[1] + 0.265 * v[0] + x[1];
   y[92] = 0.228 * v[0] + -0.265 * v[2] + v[3];
   y[93] = 0.228 * v[1] + -0.265 * v[0] + x[1];
   y[96] = 0.0019322152948007 * v[0] + -2.02185565097713e-18 * v[2] + v[3];
   y[97] = 0.0019322152948007 * v[1] + -2.02185565097713e-18 * v[0] + x[1];
   y[100] = 0.228000000119209 * v[0] + 0.264999985694885 * v[2] + v[3];
   y[101] = 0.228000000119209 * v[1] + 0.264999985694885 * v[0] + x[1];
   y[104] = 0.228000000119209 * v[0] + -0.264999985694885 * v[2] + v[3];
   y[105] = 0.228000000119209 * v[1] + -0.264999985694885 * v[0] + x[1];
   // dependent variables without operations
   y[2] = 0.13;
   y[3] = 0.100000001490116;
   y[6] = 0.125;
   y[7] = 0.100000001490116;
   y[10] = 0.125;
   y[11] = 0.100000001490116;
   y[14] = 0.125;
   y[15] = 0.100000001490116;
   y[18] = 0.13;
   y[19] = 0.100000001490116;
   y[22] = 0.24;
   y[23] = 0.0299999993294477;
   y[26] = 0.24;
   y[27] = 0.0299999993294477;
   y[30] = 0.11;
   y[31] = 0.0799999982118607;
   y[34] = 0.11;
   y[35] = 0.0799999982118607;
   y[38] = 0.11;
   y[39] = 0.0799999982118607;
   y[42] = 0.11;
   y[43] = 0.0799999982118607;
   y[46] = 0.11;
   y[47] = 0.0799999982118607;
   y[50] = 0.11;
   y[51] = 0.0799999982118607;
   y[54] = 0.11;
   y[55] = 0.0799999982118607;
   y[58] = 0.11;
   y[59] = 0.0799999982118607;
   y[62] = 0.11;
   y[63] = 0.0799999982118607;
   y[66] = 0.11;
   y[67] = 0.0799999982118607;
   y[70] = 0.11;
   y[71] = 0.0799999982118607;
   y[74] = 0.11;
   y[75] = 0.0799999982118607;
   y[78] = 0.05;
   y[79] = 0.0500000007450581;
   y[82] = 0.05;
   y[83] = 0.0500000007450581;
   y[86] = 0.275;
   y[87] = 0.100000001490116;
   y[90] = 0.1;
   y[91] = 0.100000001490116;
   y[94] = 0.1;
   y[95] = 0.100000001490116;
   y[98] = 0.136396303772926;
   y[99] = 0.387542754411697;
   y[102] = 0.100000001490116;
   y[103] = 0.100000001490116;
   y[106] = 0.100000001490116;
   y[107] = 0.100000001490116;

        





//
// environment vs. robot collisions
//

// wheel_r
if (sphere_environment_in_collision(environment,
                                    y[104],
                                    y[105],
                                    y[106],
                                    y[107]))
{
    
    
    if (sphere_environment_in_collision(environment,
                                        y[92],
                                        y[93],
                                        y[94],
                                        y[95]))
    {
        return false;
    }
    
}







//
// environment vs. robot collisions
//

// wheel_l
if (sphere_environment_in_collision(environment,
                                    y[100],
                                    y[101],
                                    y[102],
                                    y[103]))
{
    
    
    if (sphere_environment_in_collision(environment,
                                        y[88],
                                        y[89],
                                        y[90],
                                        y[91]))
    {
        return false;
    }
    
}







//
// environment vs. robot collisions
//

// base
if (sphere_environment_in_collision(environment,
                                    y[96],
                                    y[97],
                                    y[98],
                                    y[99]))
{
    
    
    if (sphere_environment_in_collision(environment,
                                        y[0],
                                        y[1],
                                        y[2],
                                        y[3]))
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[4],
                                        y[5],
                                        y[6],
                                        y[7]))
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[8],
                                        y[9],
                                        y[10],
                                        y[11]))
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[12],
                                        y[13],
                                        y[14],
                                        y[15]))
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[16],
                                        y[17],
                                        y[18],
                                        y[19]))
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[20],
                                        y[21],
                                        y[22],
                                        y[23]))
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[24],
                                        y[25],
                                        y[26],
                                        y[27]))
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[28],
                                        y[29],
                                        y[30],
                                        y[31]))
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[32],
                                        y[33],
                                        y[34],
                                        y[35]))
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[36],
                                        y[37],
                                        y[38],
                                        y[39]))
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[40],
                                        y[41],
                                        y[42],
                                        y[43]))
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[44],
                                        y[45],
                                        y[46],
                                        y[47]))
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[48],
                                        y[49],
                                        y[50],
                                        y[51]))
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[52],
                                        y[53],
                                        y[54],
                                        y[55]))
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[56],
                                        y[57],
                                        y[58],
                                        y[59]))
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[60],
                                        y[61],
                                        y[62],
                                        y[63]))
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[64],
                                        y[65],
                                        y[66],
                                        y[67]))
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[68],
                                        y[69],
                                        y[70],
                                        y[71]))
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[72],
                                        y[73],
                                        y[74],
                                        y[75]))
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[76],
                                        y[77],
                                        y[78],
                                        y[79]))
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[80],
                                        y[81],
                                        y[82],
                                        y[83]))
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[84],
                                        y[85],
                                        y[86],
                                        y[87]))
    {
        return false;
    }
    
}



//
// robot self-collisions
//





        return true;
    }

    template <std::size_t rake>
    static inline bool fkcc_attach(
        const vamp::collision::Environment<FloatVector<rake>> &environment,
        const ConfigurationBlock<rake> &x) noexcept
    {
        std::array<FloatVector<rake, 1>, 0> v;
        std::array<FloatVector<rake, 1>, 120> y;

           y[111] = cos(x[2]);
   y[112] = sin(x[2]);
   y[114] = - y[112];
   y[108] = -0.228 + x[0];
   y[0] = 0.23 * y[111] + 0.175 * y[114] + y[108];
   y[1] = 0.23 * y[112] + 0.175 * y[111] + x[1];
   y[4] = 0.23 * y[111] + 0.075 * y[114] + y[108];
   y[5] = 0.23 * y[112] + 0.075 * y[111] + x[1];
   y[8] = 0.23 * y[111] + y[108];
   y[9] = 0.23 * y[112] + x[1];
   y[12] = 0.23 * y[111] + -0.075 * y[114] + y[108];
   y[13] = 0.23 * y[112] + -0.075 * y[111] + x[1];
   y[16] = 0.23 * y[111] + -0.175 * y[114] + y[108];
   y[17] = 0.23 * y[112] + -0.175 * y[111] + x[1];
   y[20] = 0.225 * y[111] + 0.1725 * y[114] + y[108];
   y[21] = 0.225 * y[112] + 0.1725 * y[111] + x[1];
   y[24] = 0.225 * y[111] + -0.1725 * y[114] + y[108];
   y[25] = 0.225 * y[112] + -0.1725 * y[111] + x[1];
   y[28] = 0.12 * y[111] + 0.175 * y[114] + y[108];
   y[29] = 0.12 * y[112] + 0.175 * y[111] + x[1];
   y[32] = 0.12 * y[111] + -0.175 * y[114] + y[108];
   y[33] = 0.12 * y[112] + -0.175 * y[111] + x[1];
   y[36] = 0.01 * y[111] + 0.175 * y[114] + y[108];
   y[37] = 0.01 * y[112] + 0.175 * y[111] + x[1];
   y[40] = 0.01 * y[111] + -0.175 * y[114] + y[108];
   y[41] = 0.01 * y[112] + -0.175 * y[111] + x[1];
   y[44] = -0.1 * y[111] + 0.175 * y[114] + y[108];
   y[45] = -0.1 * y[112] + 0.175 * y[111] + x[1];
   y[48] = -0.1 * y[111] + -0.175 * y[114] + y[108];
   y[49] = -0.1 * y[112] + -0.175 * y[111] + x[1];
   y[52] = -0.2 * y[111] + 0.15 * y[114] + y[108];
   y[53] = -0.2 * y[112] + 0.15 * y[111] + x[1];
   y[56] = -0.2 * y[111] + -0.15 * y[114] + y[108];
   y[57] = -0.2 * y[112] + -0.15 * y[111] + x[1];
   y[60] = -0.28 * y[111] + 0.12 * y[114] + y[108];
   y[61] = -0.28 * y[112] + 0.12 * y[111] + x[1];
   y[64] = -0.28 * y[111] + 0.04 * y[114] + y[108];
   y[65] = -0.28 * y[112] + 0.04 * y[111] + x[1];
   y[68] = -0.28 * y[111] + -0.04 * y[114] + y[108];
   y[69] = -0.28 * y[112] + -0.04 * y[111] + x[1];
   y[72] = -0.28 * y[111] + -0.12 * y[114] + y[108];
   y[73] = -0.28 * y[112] + -0.12 * y[111] + x[1];
   y[76] = -0.26 * y[111] + 0.085 * y[114] + y[108];
   y[77] = -0.26 * y[112] + 0.085 * y[111] + x[1];
   y[80] = -0.26 * y[111] + -0.085 * y[114] + y[108];
   y[81] = -0.26 * y[112] + -0.085 * y[111] + x[1];
   y[84] = -0.25 * y[111] + y[108];
   y[85] = -0.25 * y[112] + x[1];
   y[88] = 0.228 * y[111] + 0.265 * y[114] + y[108];
   y[89] = 0.228 * y[112] + 0.265 * y[111] + x[1];
   y[92] = 0.228 * y[111] + -0.265 * y[114] + y[108];
   y[93] = 0.228 * y[112] + -0.265 * y[111] + x[1];
   y[96] = 0.0019322152948007 * y[111] + -2.02185565097713e-18 * y[114] + y[108];
   y[97] = 0.0019322152948007 * y[112] + -2.02185565097713e-18 * y[111] + x[1];
   y[100] = 0.228000000119209 * y[111] + 0.264999985694885 * y[114] + y[108];
   y[101] = 0.228000000119209 * y[112] + 0.264999985694885 * y[111] + x[1];
   y[104] = 0.228000000119209 * y[111] + -0.264999985694885 * y[114] + y[108];
   y[105] = 0.228000000119209 * y[112] + -0.264999985694885 * y[111] + x[1];
   // variable duplicates: 1
   y[115] = y[111];
   // dependent variables without operations
   y[2] = 0.13;
   y[3] = 0.100000001490116;
   y[6] = 0.125;
   y[7] = 0.100000001490116;
   y[10] = 0.125;
   y[11] = 0.100000001490116;
   y[14] = 0.125;
   y[15] = 0.100000001490116;
   y[18] = 0.13;
   y[19] = 0.100000001490116;
   y[22] = 0.24;
   y[23] = 0.0299999993294477;
   y[26] = 0.24;
   y[27] = 0.0299999993294477;
   y[30] = 0.11;
   y[31] = 0.0799999982118607;
   y[34] = 0.11;
   y[35] = 0.0799999982118607;
   y[38] = 0.11;
   y[39] = 0.0799999982118607;
   y[42] = 0.11;
   y[43] = 0.0799999982118607;
   y[46] = 0.11;
   y[47] = 0.0799999982118607;
   y[50] = 0.11;
   y[51] = 0.0799999982118607;
   y[54] = 0.11;
   y[55] = 0.0799999982118607;
   y[58] = 0.11;
   y[59] = 0.0799999982118607;
   y[62] = 0.11;
   y[63] = 0.0799999982118607;
   y[66] = 0.11;
   y[67] = 0.0799999982118607;
   y[70] = 0.11;
   y[71] = 0.0799999982118607;
   y[74] = 0.11;
   y[75] = 0.0799999982118607;
   y[78] = 0.05;
   y[79] = 0.0500000007450581;
   y[82] = 0.05;
   y[83] = 0.0500000007450581;
   y[86] = 0.275;
   y[87] = 0.100000001490116;
   y[90] = 0.1;
   y[91] = 0.100000001490116;
   y[94] = 0.1;
   y[95] = 0.100000001490116;
   y[98] = 0.136396303772926;
   y[99] = 0.387542754411697;
   y[102] = 0.100000001490116;
   y[103] = 0.100000001490116;
   y[106] = 0.100000001490116;
   y[107] = 0.100000001490116;
   y[109] = x[1];
   y[110] = 0.;
   y[113] = 0.;
   y[116] = 0.;
   y[117] = 0.;
   y[118] = 0.;
   y[119] = 1.;

        





//
// environment vs. robot collisions
//

// wheel_r
if (sphere_environment_in_collision(environment,
                                    y[104],
                                    y[105],
                                    y[106],
                                    y[107]))
{
    
    
    if (sphere_environment_in_collision(environment,
                                        y[92],
                                        y[93],
                                        y[94],
                                        y[95]))
    {
        return false;
    }
    
}







//
// environment vs. robot collisions
//

// wheel_l
if (sphere_environment_in_collision(environment,
                                    y[100],
                                    y[101],
                                    y[102],
                                    y[103]))
{
    
    
    if (sphere_environment_in_collision(environment,
                                        y[88],
                                        y[89],
                                        y[90],
                                        y[91]))
    {
        return false;
    }
    
}







//
// environment vs. robot collisions
//

// base
if (sphere_environment_in_collision(environment,
                                    y[96],
                                    y[97],
                                    y[98],
                                    y[99]))
{
    
    
    if (sphere_environment_in_collision(environment,
                                        y[0],
                                        y[1],
                                        y[2],
                                        y[3]))
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[4],
                                        y[5],
                                        y[6],
                                        y[7]))
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[8],
                                        y[9],
                                        y[10],
                                        y[11]))
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[12],
                                        y[13],
                                        y[14],
                                        y[15]))
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[16],
                                        y[17],
                                        y[18],
                                        y[19]))
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[20],
                                        y[21],
                                        y[22],
                                        y[23]))
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[24],
                                        y[25],
                                        y[26],
                                        y[27]))
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[28],
                                        y[29],
                                        y[30],
                                        y[31]))
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[32],
                                        y[33],
                                        y[34],
                                        y[35]))
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[36],
                                        y[37],
                                        y[38],
                                        y[39]))
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[40],
                                        y[41],
                                        y[42],
                                        y[43]))
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[44],
                                        y[45],
                                        y[46],
                                        y[47]))
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[48],
                                        y[49],
                                        y[50],
                                        y[51]))
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[52],
                                        y[53],
                                        y[54],
                                        y[55]))
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[56],
                                        y[57],
                                        y[58],
                                        y[59]))
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[60],
                                        y[61],
                                        y[62],
                                        y[63]))
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[64],
                                        y[65],
                                        y[66],
                                        y[67]))
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[68],
                                        y[69],
                                        y[70],
                                        y[71]))
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[72],
                                        y[73],
                                        y[74],
                                        y[75]))
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[76],
                                        y[77],
                                        y[78],
                                        y[79]))
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[80],
                                        y[81],
                                        y[82],
                                        y[83]))
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[84],
                                        y[85],
                                        y[86],
                                        y[87]))
    {
        return false;
    }
    
}



//
// robot self-collisions
//





        // attaching at base
        set_attachment_pose(environment, to_isometry(&y[108]));

        //
        // attachment vs. environment collisions
        //
        if (attachment_environment_collision(environment))
        {
            return false;
        }

        //
        // attachment vs. robot collisions
        //

        

        return true;
    }

    static inline auto eefk(const std::array<float, 3> &x) noexcept -> Eigen::Isometry3f
    {
        std::array<float, 0> v;
        std::array<float, 12> y;

           y[0] = -0.228 + x[0];
   y[3] = cos(x[2]);
   y[4] = sin(x[2]);
   y[6] = - y[4];
   // variable duplicates: 1
   y[7] = y[3];
   // dependent variables without operations
   y[1] = x[1];
   y[2] = 0.;
   y[5] = 0.;
   y[8] = 0.;
   y[9] = 0.;
   y[10] = 0.;
   y[11] = 1.;


        return to_isometry(y.data());
    }
};
}

// NOLINTEND(*-magic-numbers)
