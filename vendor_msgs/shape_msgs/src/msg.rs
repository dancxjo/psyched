pub mod rmw {
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[link(name = "shape_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__shape_msgs__msg__Mesh() -> *const std::ffi::c_void;
}

#[link(name = "shape_msgs__rosidl_generator_c")]
extern "C" {
    fn shape_msgs__msg__Mesh__init(msg: *mut Mesh) -> bool;
    fn shape_msgs__msg__Mesh__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Mesh>, size: usize) -> bool;
    fn shape_msgs__msg__Mesh__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Mesh>);
    fn shape_msgs__msg__Mesh__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Mesh>, out_seq: *mut rosidl_runtime_rs::Sequence<Mesh>) -> bool;
}

// Corresponds to shape_msgs__msg__Mesh
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Mesh {
    pub triangles: rosidl_runtime_rs::Sequence<crate::msg::rmw::MeshTriangle>,
    pub vertices: rosidl_runtime_rs::Sequence<geometry_msgs::msg::rmw::Point>,
}



impl Default for Mesh {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !shape_msgs__msg__Mesh__init(&mut msg as *mut _) {
        panic!("Call to shape_msgs__msg__Mesh__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Mesh {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { shape_msgs__msg__Mesh__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { shape_msgs__msg__Mesh__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { shape_msgs__msg__Mesh__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Mesh {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Mesh where Self: Sized {
  const TYPE_NAME: &'static str = "shape_msgs/msg/Mesh";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__shape_msgs__msg__Mesh() }
  }
}


#[link(name = "shape_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__shape_msgs__msg__MeshTriangle() -> *const std::ffi::c_void;
}

#[link(name = "shape_msgs__rosidl_generator_c")]
extern "C" {
    fn shape_msgs__msg__MeshTriangle__init(msg: *mut MeshTriangle) -> bool;
    fn shape_msgs__msg__MeshTriangle__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<MeshTriangle>, size: usize) -> bool;
    fn shape_msgs__msg__MeshTriangle__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<MeshTriangle>);
    fn shape_msgs__msg__MeshTriangle__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<MeshTriangle>, out_seq: *mut rosidl_runtime_rs::Sequence<MeshTriangle>) -> bool;
}

// Corresponds to shape_msgs__msg__MeshTriangle
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MeshTriangle {
    pub vertex_indices: [u32; 3],
}



impl Default for MeshTriangle {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !shape_msgs__msg__MeshTriangle__init(&mut msg as *mut _) {
        panic!("Call to shape_msgs__msg__MeshTriangle__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for MeshTriangle {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { shape_msgs__msg__MeshTriangle__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { shape_msgs__msg__MeshTriangle__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { shape_msgs__msg__MeshTriangle__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for MeshTriangle {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for MeshTriangle where Self: Sized {
  const TYPE_NAME: &'static str = "shape_msgs/msg/MeshTriangle";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__shape_msgs__msg__MeshTriangle() }
  }
}


#[link(name = "shape_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__shape_msgs__msg__Plane() -> *const std::ffi::c_void;
}

#[link(name = "shape_msgs__rosidl_generator_c")]
extern "C" {
    fn shape_msgs__msg__Plane__init(msg: *mut Plane) -> bool;
    fn shape_msgs__msg__Plane__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Plane>, size: usize) -> bool;
    fn shape_msgs__msg__Plane__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Plane>);
    fn shape_msgs__msg__Plane__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Plane>, out_seq: *mut rosidl_runtime_rs::Sequence<Plane>) -> bool;
}

// Corresponds to shape_msgs__msg__Plane
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Plane {
    pub coef: [f64; 4],
}



impl Default for Plane {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !shape_msgs__msg__Plane__init(&mut msg as *mut _) {
        panic!("Call to shape_msgs__msg__Plane__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Plane {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { shape_msgs__msg__Plane__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { shape_msgs__msg__Plane__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { shape_msgs__msg__Plane__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Plane {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Plane where Self: Sized {
  const TYPE_NAME: &'static str = "shape_msgs/msg/Plane";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__shape_msgs__msg__Plane() }
  }
}


#[link(name = "shape_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__shape_msgs__msg__SolidPrimitive() -> *const std::ffi::c_void;
}

#[link(name = "shape_msgs__rosidl_generator_c")]
extern "C" {
    fn shape_msgs__msg__SolidPrimitive__init(msg: *mut SolidPrimitive) -> bool;
    fn shape_msgs__msg__SolidPrimitive__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SolidPrimitive>, size: usize) -> bool;
    fn shape_msgs__msg__SolidPrimitive__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SolidPrimitive>);
    fn shape_msgs__msg__SolidPrimitive__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SolidPrimitive>, out_seq: *mut rosidl_runtime_rs::Sequence<SolidPrimitive>) -> bool;
}

// Corresponds to shape_msgs__msg__SolidPrimitive
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SolidPrimitive {
    pub type_: u8,
    pub dimensions: rosidl_runtime_rs::BoundedSequence<f64, 3>,
    pub polygon: geometry_msgs::msg::rmw::Polygon,
}

impl SolidPrimitive {
    pub const BOX: u8 = 1;
    pub const SPHERE: u8 = 2;
    pub const CYLINDER: u8 = 3;
    pub const CONE: u8 = 4;
    pub const PRISM: u8 = 5;
    /// The meaning of the shape dimensions: each constant defines the index in the 'dimensions' array.
    /// For type BOX, the X, Y, and Z dimensions are the length of the corresponding sides of the box.
    pub const BOX_X: u8 = 0;
    pub const BOX_Y: u8 = 1;
    pub const BOX_Z: u8 = 2;
    /// For the SPHERE type, only one component is used, and it gives the radius of the sphere.
    pub const SPHERE_RADIUS: u8 = 0;
    /// For the CYLINDER and CONE types, the center line is oriented along the Z axis.
    /// Therefore the CYLINDER_HEIGHT (CONE_HEIGHT) component of dimensions gives the
    /// height of the cylinder (cone).
    /// The CYLINDER_RADIUS (CONE_RADIUS) component of dimensions gives the radius of
    /// the base of the cylinder (cone).
    /// Cone and cylinder primitives are defined to be circular. The tip of the cone
    /// is pointing up, along +Z axis.
    pub const CYLINDER_HEIGHT: u8 = 0;
    pub const CYLINDER_RADIUS: u8 = 1;
    pub const CONE_HEIGHT: u8 = 0;
    pub const CONE_RADIUS: u8 = 1;
    /// For the type PRISM, the center line is oriented along Z axis.
    /// The PRISM_HEIGHT component of dimensions gives the
    /// height of the prism.
    /// The polygon defines the Z axis centered base of the prism.
    /// The prism is constructed by extruding the base in +Z and -Z
    /// directions by half of the PRISM_HEIGHT
    /// Only x and y fields of the points are used in the polygon.
    /// Points of the polygon are ordered counter-clockwise.
    pub const PRISM_HEIGHT: u8 = 0;
}


impl Default for SolidPrimitive {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !shape_msgs__msg__SolidPrimitive__init(&mut msg as *mut _) {
        panic!("Call to shape_msgs__msg__SolidPrimitive__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SolidPrimitive {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { shape_msgs__msg__SolidPrimitive__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { shape_msgs__msg__SolidPrimitive__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { shape_msgs__msg__SolidPrimitive__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SolidPrimitive {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SolidPrimitive where Self: Sized {
  const TYPE_NAME: &'static str = "shape_msgs/msg/SolidPrimitive";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__shape_msgs__msg__SolidPrimitive() }
  }
}


}  // mod rmw


#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Mesh {
    pub triangles: Vec<crate::msg::MeshTriangle>,
    pub vertices: Vec<geometry_msgs::msg::Point>,
}



impl Default for Mesh {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::Mesh::default())
  }
}

impl rosidl_runtime_rs::Message for Mesh {
  type RmwMsg = crate::msg::rmw::Mesh;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        triangles: msg.triangles
          .into_iter()
          .map(|elem| crate::msg::MeshTriangle::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
        vertices: msg.vertices
          .into_iter()
          .map(|elem| geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        triangles: msg.triangles
          .iter()
          .map(|elem| crate::msg::MeshTriangle::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
        vertices: msg.vertices
          .iter()
          .map(|elem| geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      triangles: msg.triangles
          .into_iter()
          .map(crate::msg::MeshTriangle::from_rmw_message)
          .collect(),
      vertices: msg.vertices
          .into_iter()
          .map(geometry_msgs::msg::Point::from_rmw_message)
          .collect(),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MeshTriangle {
    pub vertex_indices: [u32; 3],
}



impl Default for MeshTriangle {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::MeshTriangle::default())
  }
}

impl rosidl_runtime_rs::Message for MeshTriangle {
  type RmwMsg = crate::msg::rmw::MeshTriangle;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        vertex_indices: msg.vertex_indices,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        vertex_indices: msg.vertex_indices,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      vertex_indices: msg.vertex_indices,
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Plane {
    pub coef: [f64; 4],
}



impl Default for Plane {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::Plane::default())
  }
}

impl rosidl_runtime_rs::Message for Plane {
  type RmwMsg = crate::msg::rmw::Plane;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        coef: msg.coef,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        coef: msg.coef,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      coef: msg.coef,
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SolidPrimitive {
    pub type_: u8,
    pub dimensions: rosidl_runtime_rs::BoundedSequence<f64, 3>,
    pub polygon: geometry_msgs::msg::Polygon,
}

impl SolidPrimitive {
    pub const BOX: u8 = 1;
    pub const SPHERE: u8 = 2;
    pub const CYLINDER: u8 = 3;
    pub const CONE: u8 = 4;
    pub const PRISM: u8 = 5;
    /// The meaning of the shape dimensions: each constant defines the index in the 'dimensions' array.
    /// For type BOX, the X, Y, and Z dimensions are the length of the corresponding sides of the box.
    pub const BOX_X: u8 = 0;
    pub const BOX_Y: u8 = 1;
    pub const BOX_Z: u8 = 2;
    /// For the SPHERE type, only one component is used, and it gives the radius of the sphere.
    pub const SPHERE_RADIUS: u8 = 0;
    /// For the CYLINDER and CONE types, the center line is oriented along the Z axis.
    /// Therefore the CYLINDER_HEIGHT (CONE_HEIGHT) component of dimensions gives the
    /// height of the cylinder (cone).
    /// The CYLINDER_RADIUS (CONE_RADIUS) component of dimensions gives the radius of
    /// the base of the cylinder (cone).
    /// Cone and cylinder primitives are defined to be circular. The tip of the cone
    /// is pointing up, along +Z axis.
    pub const CYLINDER_HEIGHT: u8 = 0;
    pub const CYLINDER_RADIUS: u8 = 1;
    pub const CONE_HEIGHT: u8 = 0;
    pub const CONE_RADIUS: u8 = 1;
    /// For the type PRISM, the center line is oriented along Z axis.
    /// The PRISM_HEIGHT component of dimensions gives the
    /// height of the prism.
    /// The polygon defines the Z axis centered base of the prism.
    /// The prism is constructed by extruding the base in +Z and -Z
    /// directions by half of the PRISM_HEIGHT
    /// Only x and y fields of the points are used in the polygon.
    /// Points of the polygon are ordered counter-clockwise.
    pub const PRISM_HEIGHT: u8 = 0;
}


impl Default for SolidPrimitive {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::SolidPrimitive::default())
  }
}

impl rosidl_runtime_rs::Message for SolidPrimitive {
  type RmwMsg = crate::msg::rmw::SolidPrimitive;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        type_: msg.type_,
        dimensions: msg.dimensions,
        polygon: geometry_msgs::msg::Polygon::into_rmw_message(std::borrow::Cow::Owned(msg.polygon)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      type_: msg.type_,
        dimensions: msg.dimensions.clone(),
        polygon: geometry_msgs::msg::Polygon::into_rmw_message(std::borrow::Cow::Borrowed(&msg.polygon)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      type_: msg.type_,
      dimensions: msg.dimensions,
      polygon: geometry_msgs::msg::Polygon::from_rmw_message(msg.polygon),
    }
  }
}


