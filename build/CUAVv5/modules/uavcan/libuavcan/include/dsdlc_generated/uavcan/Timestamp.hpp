/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: /home/mahathi/mathur407_payload2/4.0.7/modules/uavcan/dsdl/uavcan/Timestamp.uavcan
 */

#ifndef UAVCAN_TIMESTAMP_HPP_INCLUDED
#define UAVCAN_TIMESTAMP_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

/******************************* Source text **********************************
#
# Global timestamp in microseconds, 7 bytes.
#
# Use this data type for timestamp fields in messages, like follows:
#   uavcan.Timestamp timestamp
#

uint56 UNKNOWN = 0
truncated uint56 usec     # Microseconds
******************************************************************************/

/********************* DSDL signature source definition ***********************
uavcan.Timestamp
truncated uint56 usec
******************************************************************************/

#undef usec
#undef UNKNOWN

namespace uavcan
{

template <int _tmpl>
struct UAVCAN_EXPORT Timestamp_
{
    typedef const Timestamp_<_tmpl>& ParameterType;
    typedef Timestamp_<_tmpl>& ReferenceType;

    struct ConstantTypes
    {
        typedef ::uavcan::IntegerSpec< 56, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > UNKNOWN;
    };

    struct FieldTypes
    {
        typedef ::uavcan::IntegerSpec< 56, ::uavcan::SignednessUnsigned, ::uavcan::CastModeTruncate > usec;
    };

    enum
    {
        MinBitLen
            = FieldTypes::usec::MinBitLen
    };

    enum
    {
        MaxBitLen
            = FieldTypes::usec::MaxBitLen
    };

    // Constants
    static const typename ::uavcan::StorageType< typename ConstantTypes::UNKNOWN >::Type UNKNOWN; // 0

    // Fields
    typename ::uavcan::StorageType< typename FieldTypes::usec >::Type usec;

    Timestamp_()
        : usec()
    {
        ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

#if UAVCAN_DEBUG
        /*
         * Cross-checking MaxBitLen provided by the DSDL compiler.
         * This check shall never be performed in user code because MaxBitLen value
         * actually depends on the nested types, thus it is not invariant.
         */
        ::uavcan::StaticAssert<56 == MaxBitLen>::check();
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
    // This type has no default data type ID

    static const char* getDataTypeFullName()
    {
        return "uavcan.Timestamp";
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
bool Timestamp_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        usec == rhs.usec;
}

template <int _tmpl>
bool Timestamp_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(usec, rhs.usec);
}

template <int _tmpl>
int Timestamp_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::usec::encode(self.usec, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int Timestamp_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::usec::decode(self.usec, codec,  tao_mode);
    return res;
}

/*
 * Out of line type method definitions
 */
template <int _tmpl>
::uavcan::DataTypeSignature Timestamp_<_tmpl>::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0x5BD0B5C81087E0DULL);

    FieldTypes::usec::extendDataTypeSignature(signature);

    return signature;
}

/*
 * Out of line constant definitions
 */

template <int _tmpl>
const typename ::uavcan::StorageType< typename Timestamp_<_tmpl>::ConstantTypes::UNKNOWN >::Type
    Timestamp_<_tmpl>::UNKNOWN = 0U; // 0

/*
 * Final typedef
 */
typedef Timestamp_<0> Timestamp;

// No default registration

} // Namespace uavcan

/*
 * YAML streamer specialization
 */
namespace uavcan
{

template <>
class UAVCAN_EXPORT YamlStreamer< ::uavcan::Timestamp >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::uavcan::Timestamp::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::uavcan::Timestamp >::stream(Stream& s, ::uavcan::Timestamp::ParameterType obj, const int level)
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
    s << "usec: ";
    YamlStreamer< ::uavcan::Timestamp::FieldTypes::usec >::stream(s, obj.usec, level + 1);
}

}

namespace uavcan
{

template <typename Stream>
inline Stream& operator<<(Stream& s, ::uavcan::Timestamp::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::uavcan::Timestamp >::stream(s, obj, 0);
    return s;
}

} // Namespace uavcan

#endif // UAVCAN_TIMESTAMP_HPP_INCLUDED