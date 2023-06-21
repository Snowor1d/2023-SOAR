// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from px4_msgs:msg/EstimatorAttitude.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/estimator_attitude__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
px4_msgs__msg__EstimatorAttitude__init(px4_msgs__msg__EstimatorAttitude * msg)
{
  if (!msg) {
    return false;
  }
  // timestamp
  // timestamp_sample
  // q
  // delta_q_reset
  // quat_reset_counter
  return true;
}

void
px4_msgs__msg__EstimatorAttitude__fini(px4_msgs__msg__EstimatorAttitude * msg)
{
  if (!msg) {
    return;
  }
  // timestamp
  // timestamp_sample
  // q
  // delta_q_reset
  // quat_reset_counter
}

bool
px4_msgs__msg__EstimatorAttitude__are_equal(const px4_msgs__msg__EstimatorAttitude * lhs, const px4_msgs__msg__EstimatorAttitude * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // timestamp
  if (lhs->timestamp != rhs->timestamp) {
    return false;
  }
  // timestamp_sample
  if (lhs->timestamp_sample != rhs->timestamp_sample) {
    return false;
  }
  // q
  for (size_t i = 0; i < 4; ++i) {
    if (lhs->q[i] != rhs->q[i]) {
      return false;
    }
  }
  // delta_q_reset
  for (size_t i = 0; i < 4; ++i) {
    if (lhs->delta_q_reset[i] != rhs->delta_q_reset[i]) {
      return false;
    }
  }
  // quat_reset_counter
  if (lhs->quat_reset_counter != rhs->quat_reset_counter) {
    return false;
  }
  return true;
}

bool
px4_msgs__msg__EstimatorAttitude__copy(
  const px4_msgs__msg__EstimatorAttitude * input,
  px4_msgs__msg__EstimatorAttitude * output)
{
  if (!input || !output) {
    return false;
  }
  // timestamp
  output->timestamp = input->timestamp;
  // timestamp_sample
  output->timestamp_sample = input->timestamp_sample;
  // q
  for (size_t i = 0; i < 4; ++i) {
    output->q[i] = input->q[i];
  }
  // delta_q_reset
  for (size_t i = 0; i < 4; ++i) {
    output->delta_q_reset[i] = input->delta_q_reset[i];
  }
  // quat_reset_counter
  output->quat_reset_counter = input->quat_reset_counter;
  return true;
}

px4_msgs__msg__EstimatorAttitude *
px4_msgs__msg__EstimatorAttitude__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  px4_msgs__msg__EstimatorAttitude * msg = (px4_msgs__msg__EstimatorAttitude *)allocator.allocate(sizeof(px4_msgs__msg__EstimatorAttitude), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(px4_msgs__msg__EstimatorAttitude));
  bool success = px4_msgs__msg__EstimatorAttitude__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
px4_msgs__msg__EstimatorAttitude__destroy(px4_msgs__msg__EstimatorAttitude * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    px4_msgs__msg__EstimatorAttitude__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
px4_msgs__msg__EstimatorAttitude__Sequence__init(px4_msgs__msg__EstimatorAttitude__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  px4_msgs__msg__EstimatorAttitude * data = NULL;

  if (size) {
    data = (px4_msgs__msg__EstimatorAttitude *)allocator.zero_allocate(size, sizeof(px4_msgs__msg__EstimatorAttitude), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = px4_msgs__msg__EstimatorAttitude__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        px4_msgs__msg__EstimatorAttitude__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
px4_msgs__msg__EstimatorAttitude__Sequence__fini(px4_msgs__msg__EstimatorAttitude__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      px4_msgs__msg__EstimatorAttitude__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

px4_msgs__msg__EstimatorAttitude__Sequence *
px4_msgs__msg__EstimatorAttitude__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  px4_msgs__msg__EstimatorAttitude__Sequence * array = (px4_msgs__msg__EstimatorAttitude__Sequence *)allocator.allocate(sizeof(px4_msgs__msg__EstimatorAttitude__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = px4_msgs__msg__EstimatorAttitude__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
px4_msgs__msg__EstimatorAttitude__Sequence__destroy(px4_msgs__msg__EstimatorAttitude__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    px4_msgs__msg__EstimatorAttitude__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
px4_msgs__msg__EstimatorAttitude__Sequence__are_equal(const px4_msgs__msg__EstimatorAttitude__Sequence * lhs, const px4_msgs__msg__EstimatorAttitude__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!px4_msgs__msg__EstimatorAttitude__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
px4_msgs__msg__EstimatorAttitude__Sequence__copy(
  const px4_msgs__msg__EstimatorAttitude__Sequence * input,
  px4_msgs__msg__EstimatorAttitude__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(px4_msgs__msg__EstimatorAttitude);
    px4_msgs__msg__EstimatorAttitude * data =
      (px4_msgs__msg__EstimatorAttitude *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!px4_msgs__msg__EstimatorAttitude__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          px4_msgs__msg__EstimatorAttitude__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!px4_msgs__msg__EstimatorAttitude__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}