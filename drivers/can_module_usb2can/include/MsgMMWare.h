#ifndef MSGMMWARE_H
#define MSGMMWARE_H

#include "MsgBase.h"
#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include <mmware_msgs/ObjectList.h>
#include <mmware_msgs/ClusterList.h>
#include <mmware_msgs/RadarState.h>
#include <mmware_msgs/ClusterStatus.h>
#include <mmware_msgs/Frame.h>
#include <mmware_msgs/CollisonObj.h>
#include <mmware_msgs/CollisonList.h>

namespace USB2CAN
{

typedef unsigned char ubyte;
typedef unsigned short int uword;

template <class ContainerAllocator>
struct ObjectGeneral_
{
  typedef ObjectGeneral_<ContainerAllocator> Type;

  ObjectGeneral_()
    : obj_distlong()
    , obj_distlat()
    , obj_vrellong()
    , obj_vrellat()
    , obj_dynprop()
    , obj_rcs()  {
    }
  ObjectGeneral_(const ContainerAllocator& _alloc)
    : obj_distlong(_alloc)
    , obj_distlat(_alloc)
    , obj_vrellong(_alloc)
    , obj_vrellat(_alloc)
    , obj_dynprop(_alloc)
    , obj_rcs(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Float64_<ContainerAllocator>  _obj_distlong_type;
  _obj_distlong_type obj_distlong;

   typedef  ::std_msgs::Float64_<ContainerAllocator>  _obj_distlat_type;
  _obj_distlat_type obj_distlat;

   typedef  ::std_msgs::Float64_<ContainerAllocator>  _obj_vrellong_type;
  _obj_vrellong_type obj_vrellong;

   typedef  ::std_msgs::Float64_<ContainerAllocator>  _obj_vrellat_type;
  _obj_vrellat_type obj_vrellat;

   typedef  ::std_msgs::String_<ContainerAllocator>  _obj_dynprop_type;
  _obj_dynprop_type obj_dynprop;

   typedef  ::std_msgs::Float64_<ContainerAllocator>  _obj_rcs_type;
  _obj_rcs_type obj_rcs;





  typedef boost::shared_ptr< ::USB2CAN::ObjectGeneral_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::USB2CAN::ObjectGeneral_<ContainerAllocator> const> ConstPtr;

}; // struct ObjectGeneral_

typedef ::USB2CAN::ObjectGeneral_<std::allocator<void> > ObjectGeneral;

typedef boost::shared_ptr< ::USB2CAN::ObjectGeneral > ObjectGeneralPtr;
typedef boost::shared_ptr< ::USB2CAN::ObjectGeneral const> ObjectGeneralConstPtr;

// constants requiring out of line definition

template <class ContainerAllocator>
struct Object_
{
  typedef Object_<ContainerAllocator> Type;

  Object_()
    : obj_id()
    , object_general() {
    }
  Object_(const ContainerAllocator& _alloc)
    : obj_id(_alloc)
    , object_general(_alloc) {
  (void)_alloc;
    }

   typedef  ::std_msgs::Int32_<ContainerAllocator>  _obj_id_type;
  _obj_id_type obj_id;

   typedef  ::USB2CAN::ObjectGeneral_<ContainerAllocator>  _object_general_type;
  _object_general_type object_general;

  typedef boost::shared_ptr< ::USB2CAN::Object_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::USB2CAN::Object_<ContainerAllocator> const> ConstPtr;

}; // struct Object_

typedef ::USB2CAN::Object_<std::allocator<void> > Object;

typedef boost::shared_ptr< ::USB2CAN::Object > ObjectPtr;
typedef boost::shared_ptr< ::USB2CAN::Object const> ObjectConstPtr;

// constants requiring out of line definition

template <class ContainerAllocator>
struct ObjectList_
{
  typedef ObjectList_<ContainerAllocator> Type;

  ObjectList_()
    : header()
    , object_count()
    , objects()  {
    }
  ObjectList_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , object_count(_alloc)
    , objects(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::std_msgs::Int32_<ContainerAllocator>  _object_count_type;
  _object_count_type object_count;

   typedef std::vector< ::USB2CAN::Object_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::USB2CAN::Object_<ContainerAllocator> >::other >  _objects_type;
  _objects_type objects;

  typedef boost::shared_ptr< ::USB2CAN::ObjectList_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::USB2CAN::ObjectList_<ContainerAllocator> const> ConstPtr;

}; // struct ObjectList_

typedef ::USB2CAN::ObjectList_<std::allocator<void> > ObjectList;

typedef boost::shared_ptr< ::USB2CAN::ObjectList > ObjectListPtr;
typedef boost::shared_ptr< ::USB2CAN::ObjectList const> ObjectListConstPtr;

// constants requiring out of line definition

template <class ContainerAllocator>
struct ClusterGeneral_
{
  typedef ClusterGeneral_<ContainerAllocator> Type;

  ClusterGeneral_()
    : cluster_distlong()
    , cluster_distlat()
    , cluster_vrellong()
    , cluster_vrellat()
    , cluster_dynprop()
    , cluster_rcs()  {
    }
  ClusterGeneral_(const ContainerAllocator& _alloc)
    : cluster_distlong(_alloc)
    , cluster_distlat(_alloc)
    , cluster_vrellong(_alloc)
    , cluster_vrellat(_alloc)
    , cluster_dynprop(_alloc)
    , cluster_rcs(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Float64_<ContainerAllocator>  _cluster_distlong_type;
  _cluster_distlong_type cluster_distlong;

   typedef  ::std_msgs::Float64_<ContainerAllocator>  _cluster_distlat_type;
  _cluster_distlat_type cluster_distlat;

   typedef  ::std_msgs::Float64_<ContainerAllocator>  _cluster_vrellong_type;
  _cluster_vrellong_type cluster_vrellong;

   typedef  ::std_msgs::Float64_<ContainerAllocator>  _cluster_vrellat_type;
  _cluster_vrellat_type cluster_vrellat;

   typedef  ::std_msgs::Int32_<ContainerAllocator>  _cluster_dynprop_type;
  _cluster_dynprop_type cluster_dynprop;

   typedef  ::std_msgs::Float64_<ContainerAllocator>  _cluster_rcs_type;
  _cluster_rcs_type cluster_rcs;





  typedef boost::shared_ptr< ::USB2CAN::ClusterGeneral_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::USB2CAN::ClusterGeneral_<ContainerAllocator> const> ConstPtr;

}; // struct ClusterGeneral_

typedef ::USB2CAN::ClusterGeneral_<std::allocator<void> > ClusterGeneral;

typedef boost::shared_ptr< ::USB2CAN::ClusterGeneral > ClusterGeneralPtr;
typedef boost::shared_ptr< ::USB2CAN::ClusterGeneral const> ClusterGeneralConstPtr;

// constants requiring out of line definition


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::USB2CAN::ObjectList_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::USB2CAN::ObjectList_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::USB2CAN::ObjectList_<ContainerAllocator1> & lhs, const ::USB2CAN::ObjectList_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.object_count == rhs.object_count &&
    lhs.objects == rhs.objects;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::USB2CAN::ObjectList_<ContainerAllocator1> & lhs, const ::USB2CAN::ObjectList_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}

template <class ContainerAllocator>
struct Cluster_
{
  typedef Cluster_<ContainerAllocator> Type;

  Cluster_()
    : cluster_id()
    , cluster_general() {
    }
  Cluster_(const ContainerAllocator& _alloc)
    : cluster_id(_alloc)
    , cluster_general(_alloc) {
  (void)_alloc;
    }



   typedef  ::std_msgs::Int32_<ContainerAllocator>  _cluster_id_type;
  _cluster_id_type cluster_id;

   typedef  ::USB2CAN::ClusterGeneral_<ContainerAllocator>  _cluster_general_type;
  _cluster_general_type cluster_general;

  typedef boost::shared_ptr< ::USB2CAN::Cluster_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::USB2CAN::Cluster_<ContainerAllocator> const> ConstPtr;

}; // struct Cluster_

typedef ::USB2CAN::Cluster_<std::allocator<void> > Cluster;

typedef boost::shared_ptr< ::USB2CAN::Cluster > ClusterPtr;
typedef boost::shared_ptr< ::USB2CAN::Cluster const> ClusterConstPtr;

// constants requiring out of line definition

template <class ContainerAllocator>
struct ClusterList_
{
  typedef ClusterList_<ContainerAllocator> Type;

  ClusterList_()
    : header()
    , cluster_count()
    , clusters()  {
    }
  ClusterList_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , cluster_count(_alloc)
    , clusters(_alloc)  {
  (void)_alloc;
    }

   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::std_msgs::Int32_<ContainerAllocator>  _cluster_count_type;
  _cluster_count_type cluster_count;

   typedef std::vector< ::USB2CAN::Cluster_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::USB2CAN::Cluster_<ContainerAllocator> >::other >  _clusters_type;
  _clusters_type clusters;

  typedef boost::shared_ptr< ::USB2CAN::ClusterList_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::USB2CAN::ClusterList_<ContainerAllocator> const> ConstPtr;

}; // struct ClusterList_

typedef ::USB2CAN::ClusterList_<std::allocator<void> > ClusterList;

typedef boost::shared_ptr< ::USB2CAN::ClusterList > ClusterListPtr;
typedef boost::shared_ptr< ::USB2CAN::ClusterList const> ClusterListConstPtr;

// constants requiring out of line definition


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::USB2CAN::ClusterList_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::USB2CAN::ClusterList_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::USB2CAN::ClusterList_<ContainerAllocator1> & lhs, const ::USB2CAN::ClusterList_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.cluster_count == rhs.cluster_count &&
    lhs.clusters == rhs.clusters;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::USB2CAN::ClusterList_<ContainerAllocator1> & lhs, const ::USB2CAN::ClusterList_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}

/* mmWare Cluster 0, 0x600 */
#define GET_Cluster_0_Status_Cluster_NofClustersNear(buf) (0 \
	| (ubyte)(+(ubyte)((buf[0] >> 0) & 0xff) << 0) \
)

/* mmWare Cluster1, 0x701 */
#define GET_Cluster_1_General_Cluster_DynProp(buf) (0 \
	| (ubyte)(+(ubyte)((buf[6] >> 0) & 0x07) << 0) \
)
#define GET_Cluster_1_General_Cluster_VrelLat(buf) (0 \
	| (uword)(+(uword)((buf[5] >> 0) & 0x3f) << 3) \
	| (ubyte)(+(ubyte)((buf[6] >> 5) & 0x07) << 0) \
)
#define GET_Cluster_1_General_Cluster_RCS(buf) (0 \
	| (ubyte)(+(ubyte)((buf[7] >> 0) & 0xff) << 0) \
)
#define GET_Cluster_1_General_Cluster_DistLong(buf) (0 \
	| (uword)(+(uword)((buf[1] >> 0) & 0xff) << 5) \
	| (ubyte)(+(ubyte)((buf[2] >> 3) & 0x1f) << 0) \
)
#define GET_Cluster_1_General_Cluster_ID(buf) (0 \
	| (ubyte)(+(ubyte)((buf[0] >> 0) & 0xff) << 0) \
)
#define GET_Cluster_1_General_Cluster_VrelLong(buf) (0 \
	| (uword)(+(uword)((buf[4] >> 0) & 0xff) << 2) \
	| (ubyte)(+(ubyte)((buf[5] >> 6) & 0x03) << 0) \
)
#define GET_Cluster_1_General_Cluster_DistLat(buf) (0 \
	| (uword)(+(uword)((buf[2] >> 0) & 0x03) << 8) \
	| (ubyte)(+(ubyte)((buf[3] >> 0) & 0xff) << 0) \
)

#define CALC_Cluster_1_General_Cluster_DynProp(x, fmt) \
	((x) * fmt)
#define CALC_Cluster_1_General_Cluster_VrelLat(x, fmt) \
	((x) * fmt / 4 + fmt * (-64))
#define CALC_Cluster_1_General_Cluster_RCS(x, fmt) \
	((x) * fmt / 2 + fmt * (-64))
#define CALC_Cluster_1_General_Cluster_DistLong(x, fmt) \
	((x) * fmt / 5 + fmt * (-500))
#define CALC_Cluster_1_General_Cluster_ID(x, fmt) \
	((x) * fmt)
#define CALC_Cluster_1_General_Cluster_VrelLong(x, fmt) \
	((x) * fmt / 4 + fmt * (-128))
#define CALC_Cluster_1_General_Cluster_DistLat(x, fmt) \
	((x) * fmt / 5 + fmt * (-512) / 5)

}  // namespace USB2CAN

#endif  // MSGMMWARE_H