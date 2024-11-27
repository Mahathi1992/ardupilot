/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: /home/mahathi/mathur407_payload2/4.0.7/modules/uavcan/dsdl/uavcan/equipment/ahrs/1001.MagneticFieldStrength.uavcan
 */

#ifndef UAVCAN_EQUIPMENT_AHRS_MAGNETICFIELDSTRENGTH_HPP_INCLUDED
#define UAVCAN_EQUIPMENT_AHRS_MAGNETICFIELDSTRENGTH_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

/******************************* Source text **********************************
#
# Magnetic field readings, in Gauss, in body frame.
# SI units are avoided because of float16 range limitations.
# This message is deprecated. Use the newer 1002.MagneticFieldStrength2.uavcan message.
#

float16[3] magnetic_field_ga
float16[<=9] magnetic_field_covariance
******************************************************************************/

/********************* DSDL signature source definition ***********************
uavcan.equipment.ahrs.MagneticFieldStrength
saturated float16[3] magnetic_field_ga
saturated float16[<=9] magnetic_field_covariance
******************************************************************************/

#undef magnetic_field_ga
#undef magnetic_field_covariance

namespace uavcan
{
namespace equipment
{
namespace ahrs
{

template <int _tmpl>
struct UAVCAN_EXPORT MagneticFieldStrength_
{
    typedef const MagneticFieldStrength_<_tmpl>& ParameterType;
    typedef MagneticFieldStrength_<_tmpl>& ReferenceType;

    struct ConstantTypes
    {
    };

    struct FieldTypes
    {
        typedef ::uavcan::Array< ::uavcan::FloatSpec< 16, ::uavcan::CastModeSaturate >, ::uavcan::ArrayModeStatic, 3 > magnetic_field_ga;
        typedef ::uavcan::Array< ::uavcan::FloatSpec< 16, ::uavcan::CastModeSaturate >, ::uavcan::ArrayModeDynamic, 9 > magnetic_field_covariance;
    };

    enum
    {
        MinBitLen
            = FieldTypes::magnetic_field_ga::MinBitLen
            + FieldTypes::magnetic_field_covariance::MinBitLen
    };

    enum
    {
        MaxBitLen
            = FieldTypes::magnetic_field_ga::MaxBitLen
            + FieldTypes::magnetic_field_covariance::MaxBitLen
    };

    // Constants

    // Fields
    typename ::uavcan::StorageType< typename FieldTypes::magnetic_field_ga >::Type magnetic_field_ga;
    typename ::uavcan::StorageType< typename FieldTypes::magnetic_field_covariance >::Type magnetic_field_covariance;

    MagneticFieldStrength_()
        : magnetic_field_ga()
        , magnetic_field_covariance()
    {
        ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

#if UAVCAN_DEBUG
        /*
         * Cross-checking MaxBitLen provided by the DSDL compiler.
         * This check shall never be performed in user code because MaxBitLen value
         * actually depends on the nested types, thus it is not invariant.
         */
        ::uavcan::StaticAssert<196 == MaxBitLen>::check();
#endif
    }

    bool operator==(ParameterType rhs) const;
    bool operator!=(ParameterType rhs) const { return !operator==(rhs); }

    /**
     * This comparison is based on @ref uavcan::areClose(), which ensures proper comparison of
     * floating point fields at any depth.
     */
    bool isClose(ParameterType rhs) const;

    static int encode(ParameterType self, ::uavcan::ScalarCodec& codec,
                      ::uavcan::TailArrayOptimizationMode tao_mode = ::uavcan::TailArrayOptEnabled);

    static int decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
                      ::uavcan::TailArrayOptimizationMode tao_mode = ::uavcan::TailArrayOptEnabled);

    /*
     * Static type info
     */
    enum { DataTypeKind = ::uavcan::DataTypeKindMessage };
    enum { DefaultDataTypeID = 1001 };

    static const char* getDataTypeFullName()
    {
        return "uavcan.equipment.ahrs.MagneticFieldStrength";
    }

    static void extendDataTypeSignature(::uavcan::DataTypeSignature& signature)
    {
        signature.extend(getDataTypeSignature());
    }

    static ::uavcan::DataTypeSignature getDataTypeSignature();

};

/*
 * Out of line struct method definitions
 */

template <int _tmpl>
bool MagneticFieldStrength_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        magnetic_field_ga == rhs.magnetic_field_ga &&
        magnetic_field_covariance == rhs.magnetic_field_covariance;
}

template <int _tmpl>
bool MagneticFieldStrength_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(magnetic_field_ga, rhs.magnetic_field_ga) &&
        ::uavcan::areClose(magnetic_field_covariance, rhs.magnetic_field_covariance);
}

template <int _tmpl>
int MagneticFieldStrength_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::magnetic_field_ga::encode(self.magnetic_field_ga, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::magnetic_field_covariance::encode(self.magnetic_field_covariance, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int MagneticFieldStrength_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::magnetic_field_ga::decode(self.magnetic_field_ga, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::magnetic_field_covariance::decode(self.magnetic_field_covariance, codec,  tao_mode);
    return res;
}

/*
 * Out of line type method definitions
 */
template <int _tmpl>
::uavcan::DataTypeSignature MagneticFieldStrength_<_tmpl>::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0xE2A7D4A9460BC2F2ULL);

    FieldTypes::magnetic_field_ga::extendDataTypeSignature(signature);
    FieldTypes::magnetic_field_covariance::extendDataTypeSignature(signature);

    return signature;
}

/*
 * Out of line constant definitions
 */

/*
 * Final typedef
 */
typedef MagneticFieldStrength_<0> MagneticFieldStrength;

namespace
{

const ::uavcan::DefaultDataTypeRegistrator< ::uavcan::equipment::ahrs::MagneticFieldStrength > _uavcan_gdtr_registrator_MagneticFieldStrength;

}

} // Namespace ahrs
} // Namespace equipment
} // Namespace uavcan

/*
 * YAML streamer specialization
 */
namespace uavcan
{

template <>
class UAVCAN_EXPORT YamlStreamer< ::uavcan::equipment::ahrs::MagneticFieldStrength >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::uavcan::equipment::ahrs::MagneticFieldStrength::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::uavcan::equipment::ahrs::MagneticFieldStrength >::stream(Stream& s, ::uavcan::equipment::ahrs::MagneticFieldStrength::ParameterType obj, const int level)
{
    (void)s;
    (void)obj;
    (void)level;
    if (level > 0)
    {
        s << '\n';
        for (int pos = 0; pos < level; pos++)
        {
            s << "  ";
        }
    }
    s << "magnetic_field_ga: ";
    YamlStreamer< ::uavcan::equipment::ahrs::MagneticFieldStrength::FieldTypes::magnetic_field_ga >::stream(s, obj.magnetic_field_ga, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "magnetic_field_covariance: ";
    YamlStreamer< ::uavcan::equipment::ahrs::MagneticFieldStrength::FieldTypes::magnetic_field_covariance >::stream(s, obj.magnetic_field_covariance, level + 1);
}

}

namespace uavcan
{
namespace equipment
{
namespace ahrs
{

template <typename Stream>
inline Stream& operator<<(Stream& s, ::uavcan::equipment::ahrs::MagneticFieldStrength::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::uavcan::equipment::ahrs::MagneticFieldStrength >::stream(s, obj, 0);
    return s;
}

} // Namespace ahrs
} // Namespace equipment
} // Namespace uavcan

#endif // UAVCAN_EQUIPMENT_AHRS_MAGNETICFIELDSTRENGTH_HPP_INCLUDED