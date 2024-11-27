/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: /home/mahathi/mathur_payload2/zaspixhawkap/modules/uavcan/dsdl/dronecan/sensors/hygrometer/1032.Hygrometer.uavcan
 */

#ifndef DRONECAN_SENSORS_HYGROMETER_HYGROMETER_HPP_INCLUDED
#define DRONECAN_SENSORS_HYGROMETER_HYGROMETER_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

/******************************* Source text **********************************
#
# support for hygrometer sensors
#

# temperature in degrees C, use NaN if not available
float16 temperature

# humidity as percentage
float16 humidity

# id of humidity sensor within this CAN node. Use 0 for first sensor
uint8 id
******************************************************************************/

/********************* DSDL signature source definition ***********************
dronecan.sensors.hygrometer.Hygrometer
saturated float16 temperature
saturated float16 humidity
saturated uint8 id
******************************************************************************/

#undef temperature
#undef humidity
#undef id

namespace dronecan
{
namespace sensors
{
namespace hygrometer
{

template <int _tmpl>
struct UAVCAN_EXPORT Hygrometer_
{
    typedef const Hygrometer_<_tmpl>& ParameterType;
    typedef Hygrometer_<_tmpl>& ReferenceType;

    struct ConstantTypes
    {
    };

    struct FieldTypes
    {
        typedef ::uavcan::FloatSpec< 16, ::uavcan::CastModeSaturate > temperature;
        typedef ::uavcan::FloatSpec< 16, ::uavcan::CastModeSaturate > humidity;
        typedef ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > id;
    };

    enum
    {
        MinBitLen
            = FieldTypes::temperature::MinBitLen
            + FieldTypes::humidity::MinBitLen
            + FieldTypes::id::MinBitLen
    };

    enum
    {
        MaxBitLen
            = FieldTypes::temperature::MaxBitLen
            + FieldTypes::humidity::MaxBitLen
            + FieldTypes::id::MaxBitLen
    };

    // Constants

    // Fields
    typename ::uavcan::StorageType< typename FieldTypes::temperature >::Type temperature;
    typename ::uavcan::StorageType< typename FieldTypes::humidity >::Type humidity;
    typename ::uavcan::StorageType< typename FieldTypes::id >::Type id;

    Hygrometer_()
        : temperature()
        , humidity()
        , id()
    {
        ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

#if UAVCAN_DEBUG
        /*
         * Cross-checking MaxBitLen provided by the DSDL compiler.
         * This check shall never be performed in user code because MaxBitLen value
         * actually depends on the nested types, thus it is not invariant.
         */
        ::uavcan::StaticAssert<40 == MaxBitLen>::check();
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
    enum { DefaultDataTypeID = 1032 };

    static const char* getDataTypeFullName()
    {
        return "dronecan.sensors.hygrometer.Hygrometer";
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
bool Hygrometer_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        temperature == rhs.temperature &&
        humidity == rhs.humidity &&
        id == rhs.id;
}

template <int _tmpl>
bool Hygrometer_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(temperature, rhs.temperature) &&
        ::uavcan::areClose(humidity, rhs.humidity) &&
        ::uavcan::areClose(id, rhs.id);
}

template <int _tmpl>
int Hygrometer_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::temperature::encode(self.temperature, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::humidity::encode(self.humidity, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::id::encode(self.id, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int Hygrometer_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::temperature::decode(self.temperature, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::humidity::decode(self.humidity, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::id::decode(self.id, codec,  tao_mode);
    return res;
}

/*
 * Out of line type method definitions
 */
template <int _tmpl>
::uavcan::DataTypeSignature Hygrometer_<_tmpl>::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0xCEB308892BF163E8ULL);

    FieldTypes::temperature::extendDataTypeSignature(signature);
    FieldTypes::humidity::extendDataTypeSignature(signature);
    FieldTypes::id::extendDataTypeSignature(signature);

    return signature;
}

/*
 * Out of line constant definitions
 */

/*
 * Final typedef
 */
typedef Hygrometer_<0> Hygrometer;

namespace
{

const ::uavcan::DefaultDataTypeRegistrator< ::dronecan::sensors::hygrometer::Hygrometer > _uavcan_gdtr_registrator_Hygrometer;

}

} // Namespace hygrometer
} // Namespace sensors
} // Namespace dronecan

/*
 * YAML streamer specialization
 */
namespace uavcan
{

template <>
class UAVCAN_EXPORT YamlStreamer< ::dronecan::sensors::hygrometer::Hygrometer >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::dronecan::sensors::hygrometer::Hygrometer::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::dronecan::sensors::hygrometer::Hygrometer >::stream(Stream& s, ::dronecan::sensors::hygrometer::Hygrometer::ParameterType obj, const int level)
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
    s << "temperature: ";
    YamlStreamer< ::dronecan::sensors::hygrometer::Hygrometer::FieldTypes::temperature >::stream(s, obj.temperature, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "humidity: ";
    YamlStreamer< ::dronecan::sensors::hygrometer::Hygrometer::FieldTypes::humidity >::stream(s, obj.humidity, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "id: ";
    YamlStreamer< ::dronecan::sensors::hygrometer::Hygrometer::FieldTypes::id >::stream(s, obj.id, level + 1);
}

}

namespace dronecan
{
namespace sensors
{
namespace hygrometer
{

template <typename Stream>
inline Stream& operator<<(Stream& s, ::dronecan::sensors::hygrometer::Hygrometer::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::dronecan::sensors::hygrometer::Hygrometer >::stream(s, obj, 0);
    return s;
}

} // Namespace hygrometer
} // Namespace sensors
} // Namespace dronecan

#endif // DRONECAN_SENSORS_HYGROMETER_HYGROMETER_HPP_INCLUDED