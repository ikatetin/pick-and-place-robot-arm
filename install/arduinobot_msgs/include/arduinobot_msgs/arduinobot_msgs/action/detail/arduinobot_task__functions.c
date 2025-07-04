// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from arduinobot_msgs:action/ArduinobotTask.idl
// generated code does not contain a copyright notice
#include "arduinobot_msgs/action/detail/arduinobot_task__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `arm_goal`
// Member `gripper_goal`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
arduinobot_msgs__action__ArduinobotTask_Goal__init(arduinobot_msgs__action__ArduinobotTask_Goal * msg)
{
  if (!msg) {
    return false;
  }
  // task_number
  // arm_goal
  if (!rosidl_runtime_c__double__Sequence__init(&msg->arm_goal, 0)) {
    arduinobot_msgs__action__ArduinobotTask_Goal__fini(msg);
    return false;
  }
  // gripper_goal
  if (!rosidl_runtime_c__double__Sequence__init(&msg->gripper_goal, 0)) {
    arduinobot_msgs__action__ArduinobotTask_Goal__fini(msg);
    return false;
  }
  return true;
}

void
arduinobot_msgs__action__ArduinobotTask_Goal__fini(arduinobot_msgs__action__ArduinobotTask_Goal * msg)
{
  if (!msg) {
    return;
  }
  // task_number
  // arm_goal
  rosidl_runtime_c__double__Sequence__fini(&msg->arm_goal);
  // gripper_goal
  rosidl_runtime_c__double__Sequence__fini(&msg->gripper_goal);
}

bool
arduinobot_msgs__action__ArduinobotTask_Goal__are_equal(const arduinobot_msgs__action__ArduinobotTask_Goal * lhs, const arduinobot_msgs__action__ArduinobotTask_Goal * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // task_number
  if (lhs->task_number != rhs->task_number) {
    return false;
  }
  // arm_goal
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->arm_goal), &(rhs->arm_goal)))
  {
    return false;
  }
  // gripper_goal
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->gripper_goal), &(rhs->gripper_goal)))
  {
    return false;
  }
  return true;
}

bool
arduinobot_msgs__action__ArduinobotTask_Goal__copy(
  const arduinobot_msgs__action__ArduinobotTask_Goal * input,
  arduinobot_msgs__action__ArduinobotTask_Goal * output)
{
  if (!input || !output) {
    return false;
  }
  // task_number
  output->task_number = input->task_number;
  // arm_goal
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->arm_goal), &(output->arm_goal)))
  {
    return false;
  }
  // gripper_goal
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->gripper_goal), &(output->gripper_goal)))
  {
    return false;
  }
  return true;
}

arduinobot_msgs__action__ArduinobotTask_Goal *
arduinobot_msgs__action__ArduinobotTask_Goal__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arduinobot_msgs__action__ArduinobotTask_Goal * msg = (arduinobot_msgs__action__ArduinobotTask_Goal *)allocator.allocate(sizeof(arduinobot_msgs__action__ArduinobotTask_Goal), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(arduinobot_msgs__action__ArduinobotTask_Goal));
  bool success = arduinobot_msgs__action__ArduinobotTask_Goal__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
arduinobot_msgs__action__ArduinobotTask_Goal__destroy(arduinobot_msgs__action__ArduinobotTask_Goal * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    arduinobot_msgs__action__ArduinobotTask_Goal__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
arduinobot_msgs__action__ArduinobotTask_Goal__Sequence__init(arduinobot_msgs__action__ArduinobotTask_Goal__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arduinobot_msgs__action__ArduinobotTask_Goal * data = NULL;

  if (size) {
    data = (arduinobot_msgs__action__ArduinobotTask_Goal *)allocator.zero_allocate(size, sizeof(arduinobot_msgs__action__ArduinobotTask_Goal), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = arduinobot_msgs__action__ArduinobotTask_Goal__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        arduinobot_msgs__action__ArduinobotTask_Goal__fini(&data[i - 1]);
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
arduinobot_msgs__action__ArduinobotTask_Goal__Sequence__fini(arduinobot_msgs__action__ArduinobotTask_Goal__Sequence * array)
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
      arduinobot_msgs__action__ArduinobotTask_Goal__fini(&array->data[i]);
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

arduinobot_msgs__action__ArduinobotTask_Goal__Sequence *
arduinobot_msgs__action__ArduinobotTask_Goal__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arduinobot_msgs__action__ArduinobotTask_Goal__Sequence * array = (arduinobot_msgs__action__ArduinobotTask_Goal__Sequence *)allocator.allocate(sizeof(arduinobot_msgs__action__ArduinobotTask_Goal__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = arduinobot_msgs__action__ArduinobotTask_Goal__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
arduinobot_msgs__action__ArduinobotTask_Goal__Sequence__destroy(arduinobot_msgs__action__ArduinobotTask_Goal__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    arduinobot_msgs__action__ArduinobotTask_Goal__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
arduinobot_msgs__action__ArduinobotTask_Goal__Sequence__are_equal(const arduinobot_msgs__action__ArduinobotTask_Goal__Sequence * lhs, const arduinobot_msgs__action__ArduinobotTask_Goal__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!arduinobot_msgs__action__ArduinobotTask_Goal__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
arduinobot_msgs__action__ArduinobotTask_Goal__Sequence__copy(
  const arduinobot_msgs__action__ArduinobotTask_Goal__Sequence * input,
  arduinobot_msgs__action__ArduinobotTask_Goal__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(arduinobot_msgs__action__ArduinobotTask_Goal);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    arduinobot_msgs__action__ArduinobotTask_Goal * data =
      (arduinobot_msgs__action__ArduinobotTask_Goal *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!arduinobot_msgs__action__ArduinobotTask_Goal__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          arduinobot_msgs__action__ArduinobotTask_Goal__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!arduinobot_msgs__action__ArduinobotTask_Goal__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
arduinobot_msgs__action__ArduinobotTask_Result__init(arduinobot_msgs__action__ArduinobotTask_Result * msg)
{
  if (!msg) {
    return false;
  }
  // success
  return true;
}

void
arduinobot_msgs__action__ArduinobotTask_Result__fini(arduinobot_msgs__action__ArduinobotTask_Result * msg)
{
  if (!msg) {
    return;
  }
  // success
}

bool
arduinobot_msgs__action__ArduinobotTask_Result__are_equal(const arduinobot_msgs__action__ArduinobotTask_Result * lhs, const arduinobot_msgs__action__ArduinobotTask_Result * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  return true;
}

bool
arduinobot_msgs__action__ArduinobotTask_Result__copy(
  const arduinobot_msgs__action__ArduinobotTask_Result * input,
  arduinobot_msgs__action__ArduinobotTask_Result * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  return true;
}

arduinobot_msgs__action__ArduinobotTask_Result *
arduinobot_msgs__action__ArduinobotTask_Result__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arduinobot_msgs__action__ArduinobotTask_Result * msg = (arduinobot_msgs__action__ArduinobotTask_Result *)allocator.allocate(sizeof(arduinobot_msgs__action__ArduinobotTask_Result), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(arduinobot_msgs__action__ArduinobotTask_Result));
  bool success = arduinobot_msgs__action__ArduinobotTask_Result__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
arduinobot_msgs__action__ArduinobotTask_Result__destroy(arduinobot_msgs__action__ArduinobotTask_Result * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    arduinobot_msgs__action__ArduinobotTask_Result__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
arduinobot_msgs__action__ArduinobotTask_Result__Sequence__init(arduinobot_msgs__action__ArduinobotTask_Result__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arduinobot_msgs__action__ArduinobotTask_Result * data = NULL;

  if (size) {
    data = (arduinobot_msgs__action__ArduinobotTask_Result *)allocator.zero_allocate(size, sizeof(arduinobot_msgs__action__ArduinobotTask_Result), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = arduinobot_msgs__action__ArduinobotTask_Result__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        arduinobot_msgs__action__ArduinobotTask_Result__fini(&data[i - 1]);
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
arduinobot_msgs__action__ArduinobotTask_Result__Sequence__fini(arduinobot_msgs__action__ArduinobotTask_Result__Sequence * array)
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
      arduinobot_msgs__action__ArduinobotTask_Result__fini(&array->data[i]);
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

arduinobot_msgs__action__ArduinobotTask_Result__Sequence *
arduinobot_msgs__action__ArduinobotTask_Result__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arduinobot_msgs__action__ArduinobotTask_Result__Sequence * array = (arduinobot_msgs__action__ArduinobotTask_Result__Sequence *)allocator.allocate(sizeof(arduinobot_msgs__action__ArduinobotTask_Result__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = arduinobot_msgs__action__ArduinobotTask_Result__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
arduinobot_msgs__action__ArduinobotTask_Result__Sequence__destroy(arduinobot_msgs__action__ArduinobotTask_Result__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    arduinobot_msgs__action__ArduinobotTask_Result__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
arduinobot_msgs__action__ArduinobotTask_Result__Sequence__are_equal(const arduinobot_msgs__action__ArduinobotTask_Result__Sequence * lhs, const arduinobot_msgs__action__ArduinobotTask_Result__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!arduinobot_msgs__action__ArduinobotTask_Result__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
arduinobot_msgs__action__ArduinobotTask_Result__Sequence__copy(
  const arduinobot_msgs__action__ArduinobotTask_Result__Sequence * input,
  arduinobot_msgs__action__ArduinobotTask_Result__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(arduinobot_msgs__action__ArduinobotTask_Result);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    arduinobot_msgs__action__ArduinobotTask_Result * data =
      (arduinobot_msgs__action__ArduinobotTask_Result *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!arduinobot_msgs__action__ArduinobotTask_Result__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          arduinobot_msgs__action__ArduinobotTask_Result__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!arduinobot_msgs__action__ArduinobotTask_Result__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
arduinobot_msgs__action__ArduinobotTask_Feedback__init(arduinobot_msgs__action__ArduinobotTask_Feedback * msg)
{
  if (!msg) {
    return false;
  }
  // percentage
  return true;
}

void
arduinobot_msgs__action__ArduinobotTask_Feedback__fini(arduinobot_msgs__action__ArduinobotTask_Feedback * msg)
{
  if (!msg) {
    return;
  }
  // percentage
}

bool
arduinobot_msgs__action__ArduinobotTask_Feedback__are_equal(const arduinobot_msgs__action__ArduinobotTask_Feedback * lhs, const arduinobot_msgs__action__ArduinobotTask_Feedback * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // percentage
  if (lhs->percentage != rhs->percentage) {
    return false;
  }
  return true;
}

bool
arduinobot_msgs__action__ArduinobotTask_Feedback__copy(
  const arduinobot_msgs__action__ArduinobotTask_Feedback * input,
  arduinobot_msgs__action__ArduinobotTask_Feedback * output)
{
  if (!input || !output) {
    return false;
  }
  // percentage
  output->percentage = input->percentage;
  return true;
}

arduinobot_msgs__action__ArduinobotTask_Feedback *
arduinobot_msgs__action__ArduinobotTask_Feedback__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arduinobot_msgs__action__ArduinobotTask_Feedback * msg = (arduinobot_msgs__action__ArduinobotTask_Feedback *)allocator.allocate(sizeof(arduinobot_msgs__action__ArduinobotTask_Feedback), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(arduinobot_msgs__action__ArduinobotTask_Feedback));
  bool success = arduinobot_msgs__action__ArduinobotTask_Feedback__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
arduinobot_msgs__action__ArduinobotTask_Feedback__destroy(arduinobot_msgs__action__ArduinobotTask_Feedback * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    arduinobot_msgs__action__ArduinobotTask_Feedback__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
arduinobot_msgs__action__ArduinobotTask_Feedback__Sequence__init(arduinobot_msgs__action__ArduinobotTask_Feedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arduinobot_msgs__action__ArduinobotTask_Feedback * data = NULL;

  if (size) {
    data = (arduinobot_msgs__action__ArduinobotTask_Feedback *)allocator.zero_allocate(size, sizeof(arduinobot_msgs__action__ArduinobotTask_Feedback), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = arduinobot_msgs__action__ArduinobotTask_Feedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        arduinobot_msgs__action__ArduinobotTask_Feedback__fini(&data[i - 1]);
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
arduinobot_msgs__action__ArduinobotTask_Feedback__Sequence__fini(arduinobot_msgs__action__ArduinobotTask_Feedback__Sequence * array)
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
      arduinobot_msgs__action__ArduinobotTask_Feedback__fini(&array->data[i]);
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

arduinobot_msgs__action__ArduinobotTask_Feedback__Sequence *
arduinobot_msgs__action__ArduinobotTask_Feedback__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arduinobot_msgs__action__ArduinobotTask_Feedback__Sequence * array = (arduinobot_msgs__action__ArduinobotTask_Feedback__Sequence *)allocator.allocate(sizeof(arduinobot_msgs__action__ArduinobotTask_Feedback__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = arduinobot_msgs__action__ArduinobotTask_Feedback__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
arduinobot_msgs__action__ArduinobotTask_Feedback__Sequence__destroy(arduinobot_msgs__action__ArduinobotTask_Feedback__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    arduinobot_msgs__action__ArduinobotTask_Feedback__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
arduinobot_msgs__action__ArduinobotTask_Feedback__Sequence__are_equal(const arduinobot_msgs__action__ArduinobotTask_Feedback__Sequence * lhs, const arduinobot_msgs__action__ArduinobotTask_Feedback__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!arduinobot_msgs__action__ArduinobotTask_Feedback__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
arduinobot_msgs__action__ArduinobotTask_Feedback__Sequence__copy(
  const arduinobot_msgs__action__ArduinobotTask_Feedback__Sequence * input,
  arduinobot_msgs__action__ArduinobotTask_Feedback__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(arduinobot_msgs__action__ArduinobotTask_Feedback);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    arduinobot_msgs__action__ArduinobotTask_Feedback * data =
      (arduinobot_msgs__action__ArduinobotTask_Feedback *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!arduinobot_msgs__action__ArduinobotTask_Feedback__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          arduinobot_msgs__action__ArduinobotTask_Feedback__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!arduinobot_msgs__action__ArduinobotTask_Feedback__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `goal`
// already included above
// #include "arduinobot_msgs/action/detail/arduinobot_task__functions.h"

bool
arduinobot_msgs__action__ArduinobotTask_SendGoal_Request__init(arduinobot_msgs__action__ArduinobotTask_SendGoal_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    arduinobot_msgs__action__ArduinobotTask_SendGoal_Request__fini(msg);
    return false;
  }
  // goal
  if (!arduinobot_msgs__action__ArduinobotTask_Goal__init(&msg->goal)) {
    arduinobot_msgs__action__ArduinobotTask_SendGoal_Request__fini(msg);
    return false;
  }
  return true;
}

void
arduinobot_msgs__action__ArduinobotTask_SendGoal_Request__fini(arduinobot_msgs__action__ArduinobotTask_SendGoal_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // goal
  arduinobot_msgs__action__ArduinobotTask_Goal__fini(&msg->goal);
}

bool
arduinobot_msgs__action__ArduinobotTask_SendGoal_Request__are_equal(const arduinobot_msgs__action__ArduinobotTask_SendGoal_Request * lhs, const arduinobot_msgs__action__ArduinobotTask_SendGoal_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // goal
  if (!arduinobot_msgs__action__ArduinobotTask_Goal__are_equal(
      &(lhs->goal), &(rhs->goal)))
  {
    return false;
  }
  return true;
}

bool
arduinobot_msgs__action__ArduinobotTask_SendGoal_Request__copy(
  const arduinobot_msgs__action__ArduinobotTask_SendGoal_Request * input,
  arduinobot_msgs__action__ArduinobotTask_SendGoal_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // goal
  if (!arduinobot_msgs__action__ArduinobotTask_Goal__copy(
      &(input->goal), &(output->goal)))
  {
    return false;
  }
  return true;
}

arduinobot_msgs__action__ArduinobotTask_SendGoal_Request *
arduinobot_msgs__action__ArduinobotTask_SendGoal_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arduinobot_msgs__action__ArduinobotTask_SendGoal_Request * msg = (arduinobot_msgs__action__ArduinobotTask_SendGoal_Request *)allocator.allocate(sizeof(arduinobot_msgs__action__ArduinobotTask_SendGoal_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(arduinobot_msgs__action__ArduinobotTask_SendGoal_Request));
  bool success = arduinobot_msgs__action__ArduinobotTask_SendGoal_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
arduinobot_msgs__action__ArduinobotTask_SendGoal_Request__destroy(arduinobot_msgs__action__ArduinobotTask_SendGoal_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    arduinobot_msgs__action__ArduinobotTask_SendGoal_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
arduinobot_msgs__action__ArduinobotTask_SendGoal_Request__Sequence__init(arduinobot_msgs__action__ArduinobotTask_SendGoal_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arduinobot_msgs__action__ArduinobotTask_SendGoal_Request * data = NULL;

  if (size) {
    data = (arduinobot_msgs__action__ArduinobotTask_SendGoal_Request *)allocator.zero_allocate(size, sizeof(arduinobot_msgs__action__ArduinobotTask_SendGoal_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = arduinobot_msgs__action__ArduinobotTask_SendGoal_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        arduinobot_msgs__action__ArduinobotTask_SendGoal_Request__fini(&data[i - 1]);
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
arduinobot_msgs__action__ArduinobotTask_SendGoal_Request__Sequence__fini(arduinobot_msgs__action__ArduinobotTask_SendGoal_Request__Sequence * array)
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
      arduinobot_msgs__action__ArduinobotTask_SendGoal_Request__fini(&array->data[i]);
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

arduinobot_msgs__action__ArduinobotTask_SendGoal_Request__Sequence *
arduinobot_msgs__action__ArduinobotTask_SendGoal_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arduinobot_msgs__action__ArduinobotTask_SendGoal_Request__Sequence * array = (arduinobot_msgs__action__ArduinobotTask_SendGoal_Request__Sequence *)allocator.allocate(sizeof(arduinobot_msgs__action__ArduinobotTask_SendGoal_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = arduinobot_msgs__action__ArduinobotTask_SendGoal_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
arduinobot_msgs__action__ArduinobotTask_SendGoal_Request__Sequence__destroy(arduinobot_msgs__action__ArduinobotTask_SendGoal_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    arduinobot_msgs__action__ArduinobotTask_SendGoal_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
arduinobot_msgs__action__ArduinobotTask_SendGoal_Request__Sequence__are_equal(const arduinobot_msgs__action__ArduinobotTask_SendGoal_Request__Sequence * lhs, const arduinobot_msgs__action__ArduinobotTask_SendGoal_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!arduinobot_msgs__action__ArduinobotTask_SendGoal_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
arduinobot_msgs__action__ArduinobotTask_SendGoal_Request__Sequence__copy(
  const arduinobot_msgs__action__ArduinobotTask_SendGoal_Request__Sequence * input,
  arduinobot_msgs__action__ArduinobotTask_SendGoal_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(arduinobot_msgs__action__ArduinobotTask_SendGoal_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    arduinobot_msgs__action__ArduinobotTask_SendGoal_Request * data =
      (arduinobot_msgs__action__ArduinobotTask_SendGoal_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!arduinobot_msgs__action__ArduinobotTask_SendGoal_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          arduinobot_msgs__action__ArduinobotTask_SendGoal_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!arduinobot_msgs__action__ArduinobotTask_SendGoal_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
arduinobot_msgs__action__ArduinobotTask_SendGoal_Response__init(arduinobot_msgs__action__ArduinobotTask_SendGoal_Response * msg)
{
  if (!msg) {
    return false;
  }
  // accepted
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    arduinobot_msgs__action__ArduinobotTask_SendGoal_Response__fini(msg);
    return false;
  }
  return true;
}

void
arduinobot_msgs__action__ArduinobotTask_SendGoal_Response__fini(arduinobot_msgs__action__ArduinobotTask_SendGoal_Response * msg)
{
  if (!msg) {
    return;
  }
  // accepted
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
}

bool
arduinobot_msgs__action__ArduinobotTask_SendGoal_Response__are_equal(const arduinobot_msgs__action__ArduinobotTask_SendGoal_Response * lhs, const arduinobot_msgs__action__ArduinobotTask_SendGoal_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // accepted
  if (lhs->accepted != rhs->accepted) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->stamp), &(rhs->stamp)))
  {
    return false;
  }
  return true;
}

bool
arduinobot_msgs__action__ArduinobotTask_SendGoal_Response__copy(
  const arduinobot_msgs__action__ArduinobotTask_SendGoal_Response * input,
  arduinobot_msgs__action__ArduinobotTask_SendGoal_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // accepted
  output->accepted = input->accepted;
  // stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->stamp), &(output->stamp)))
  {
    return false;
  }
  return true;
}

arduinobot_msgs__action__ArduinobotTask_SendGoal_Response *
arduinobot_msgs__action__ArduinobotTask_SendGoal_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arduinobot_msgs__action__ArduinobotTask_SendGoal_Response * msg = (arduinobot_msgs__action__ArduinobotTask_SendGoal_Response *)allocator.allocate(sizeof(arduinobot_msgs__action__ArduinobotTask_SendGoal_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(arduinobot_msgs__action__ArduinobotTask_SendGoal_Response));
  bool success = arduinobot_msgs__action__ArduinobotTask_SendGoal_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
arduinobot_msgs__action__ArduinobotTask_SendGoal_Response__destroy(arduinobot_msgs__action__ArduinobotTask_SendGoal_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    arduinobot_msgs__action__ArduinobotTask_SendGoal_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
arduinobot_msgs__action__ArduinobotTask_SendGoal_Response__Sequence__init(arduinobot_msgs__action__ArduinobotTask_SendGoal_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arduinobot_msgs__action__ArduinobotTask_SendGoal_Response * data = NULL;

  if (size) {
    data = (arduinobot_msgs__action__ArduinobotTask_SendGoal_Response *)allocator.zero_allocate(size, sizeof(arduinobot_msgs__action__ArduinobotTask_SendGoal_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = arduinobot_msgs__action__ArduinobotTask_SendGoal_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        arduinobot_msgs__action__ArduinobotTask_SendGoal_Response__fini(&data[i - 1]);
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
arduinobot_msgs__action__ArduinobotTask_SendGoal_Response__Sequence__fini(arduinobot_msgs__action__ArduinobotTask_SendGoal_Response__Sequence * array)
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
      arduinobot_msgs__action__ArduinobotTask_SendGoal_Response__fini(&array->data[i]);
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

arduinobot_msgs__action__ArduinobotTask_SendGoal_Response__Sequence *
arduinobot_msgs__action__ArduinobotTask_SendGoal_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arduinobot_msgs__action__ArduinobotTask_SendGoal_Response__Sequence * array = (arduinobot_msgs__action__ArduinobotTask_SendGoal_Response__Sequence *)allocator.allocate(sizeof(arduinobot_msgs__action__ArduinobotTask_SendGoal_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = arduinobot_msgs__action__ArduinobotTask_SendGoal_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
arduinobot_msgs__action__ArduinobotTask_SendGoal_Response__Sequence__destroy(arduinobot_msgs__action__ArduinobotTask_SendGoal_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    arduinobot_msgs__action__ArduinobotTask_SendGoal_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
arduinobot_msgs__action__ArduinobotTask_SendGoal_Response__Sequence__are_equal(const arduinobot_msgs__action__ArduinobotTask_SendGoal_Response__Sequence * lhs, const arduinobot_msgs__action__ArduinobotTask_SendGoal_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!arduinobot_msgs__action__ArduinobotTask_SendGoal_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
arduinobot_msgs__action__ArduinobotTask_SendGoal_Response__Sequence__copy(
  const arduinobot_msgs__action__ArduinobotTask_SendGoal_Response__Sequence * input,
  arduinobot_msgs__action__ArduinobotTask_SendGoal_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(arduinobot_msgs__action__ArduinobotTask_SendGoal_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    arduinobot_msgs__action__ArduinobotTask_SendGoal_Response * data =
      (arduinobot_msgs__action__ArduinobotTask_SendGoal_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!arduinobot_msgs__action__ArduinobotTask_SendGoal_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          arduinobot_msgs__action__ArduinobotTask_SendGoal_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!arduinobot_msgs__action__ArduinobotTask_SendGoal_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `info`
#include "service_msgs/msg/detail/service_event_info__functions.h"
// Member `request`
// Member `response`
// already included above
// #include "arduinobot_msgs/action/detail/arduinobot_task__functions.h"

bool
arduinobot_msgs__action__ArduinobotTask_SendGoal_Event__init(arduinobot_msgs__action__ArduinobotTask_SendGoal_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    arduinobot_msgs__action__ArduinobotTask_SendGoal_Event__fini(msg);
    return false;
  }
  // request
  if (!arduinobot_msgs__action__ArduinobotTask_SendGoal_Request__Sequence__init(&msg->request, 0)) {
    arduinobot_msgs__action__ArduinobotTask_SendGoal_Event__fini(msg);
    return false;
  }
  // response
  if (!arduinobot_msgs__action__ArduinobotTask_SendGoal_Response__Sequence__init(&msg->response, 0)) {
    arduinobot_msgs__action__ArduinobotTask_SendGoal_Event__fini(msg);
    return false;
  }
  return true;
}

void
arduinobot_msgs__action__ArduinobotTask_SendGoal_Event__fini(arduinobot_msgs__action__ArduinobotTask_SendGoal_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  arduinobot_msgs__action__ArduinobotTask_SendGoal_Request__Sequence__fini(&msg->request);
  // response
  arduinobot_msgs__action__ArduinobotTask_SendGoal_Response__Sequence__fini(&msg->response);
}

bool
arduinobot_msgs__action__ArduinobotTask_SendGoal_Event__are_equal(const arduinobot_msgs__action__ArduinobotTask_SendGoal_Event * lhs, const arduinobot_msgs__action__ArduinobotTask_SendGoal_Event * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__are_equal(
      &(lhs->info), &(rhs->info)))
  {
    return false;
  }
  // request
  if (!arduinobot_msgs__action__ArduinobotTask_SendGoal_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!arduinobot_msgs__action__ArduinobotTask_SendGoal_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
arduinobot_msgs__action__ArduinobotTask_SendGoal_Event__copy(
  const arduinobot_msgs__action__ArduinobotTask_SendGoal_Event * input,
  arduinobot_msgs__action__ArduinobotTask_SendGoal_Event * output)
{
  if (!input || !output) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__copy(
      &(input->info), &(output->info)))
  {
    return false;
  }
  // request
  if (!arduinobot_msgs__action__ArduinobotTask_SendGoal_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!arduinobot_msgs__action__ArduinobotTask_SendGoal_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

arduinobot_msgs__action__ArduinobotTask_SendGoal_Event *
arduinobot_msgs__action__ArduinobotTask_SendGoal_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arduinobot_msgs__action__ArduinobotTask_SendGoal_Event * msg = (arduinobot_msgs__action__ArduinobotTask_SendGoal_Event *)allocator.allocate(sizeof(arduinobot_msgs__action__ArduinobotTask_SendGoal_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(arduinobot_msgs__action__ArduinobotTask_SendGoal_Event));
  bool success = arduinobot_msgs__action__ArduinobotTask_SendGoal_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
arduinobot_msgs__action__ArduinobotTask_SendGoal_Event__destroy(arduinobot_msgs__action__ArduinobotTask_SendGoal_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    arduinobot_msgs__action__ArduinobotTask_SendGoal_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
arduinobot_msgs__action__ArduinobotTask_SendGoal_Event__Sequence__init(arduinobot_msgs__action__ArduinobotTask_SendGoal_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arduinobot_msgs__action__ArduinobotTask_SendGoal_Event * data = NULL;

  if (size) {
    data = (arduinobot_msgs__action__ArduinobotTask_SendGoal_Event *)allocator.zero_allocate(size, sizeof(arduinobot_msgs__action__ArduinobotTask_SendGoal_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = arduinobot_msgs__action__ArduinobotTask_SendGoal_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        arduinobot_msgs__action__ArduinobotTask_SendGoal_Event__fini(&data[i - 1]);
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
arduinobot_msgs__action__ArduinobotTask_SendGoal_Event__Sequence__fini(arduinobot_msgs__action__ArduinobotTask_SendGoal_Event__Sequence * array)
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
      arduinobot_msgs__action__ArduinobotTask_SendGoal_Event__fini(&array->data[i]);
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

arduinobot_msgs__action__ArduinobotTask_SendGoal_Event__Sequence *
arduinobot_msgs__action__ArduinobotTask_SendGoal_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arduinobot_msgs__action__ArduinobotTask_SendGoal_Event__Sequence * array = (arduinobot_msgs__action__ArduinobotTask_SendGoal_Event__Sequence *)allocator.allocate(sizeof(arduinobot_msgs__action__ArduinobotTask_SendGoal_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = arduinobot_msgs__action__ArduinobotTask_SendGoal_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
arduinobot_msgs__action__ArduinobotTask_SendGoal_Event__Sequence__destroy(arduinobot_msgs__action__ArduinobotTask_SendGoal_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    arduinobot_msgs__action__ArduinobotTask_SendGoal_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
arduinobot_msgs__action__ArduinobotTask_SendGoal_Event__Sequence__are_equal(const arduinobot_msgs__action__ArduinobotTask_SendGoal_Event__Sequence * lhs, const arduinobot_msgs__action__ArduinobotTask_SendGoal_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!arduinobot_msgs__action__ArduinobotTask_SendGoal_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
arduinobot_msgs__action__ArduinobotTask_SendGoal_Event__Sequence__copy(
  const arduinobot_msgs__action__ArduinobotTask_SendGoal_Event__Sequence * input,
  arduinobot_msgs__action__ArduinobotTask_SendGoal_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(arduinobot_msgs__action__ArduinobotTask_SendGoal_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    arduinobot_msgs__action__ArduinobotTask_SendGoal_Event * data =
      (arduinobot_msgs__action__ArduinobotTask_SendGoal_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!arduinobot_msgs__action__ArduinobotTask_SendGoal_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          arduinobot_msgs__action__ArduinobotTask_SendGoal_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!arduinobot_msgs__action__ArduinobotTask_SendGoal_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"

bool
arduinobot_msgs__action__ArduinobotTask_GetResult_Request__init(arduinobot_msgs__action__ArduinobotTask_GetResult_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    arduinobot_msgs__action__ArduinobotTask_GetResult_Request__fini(msg);
    return false;
  }
  return true;
}

void
arduinobot_msgs__action__ArduinobotTask_GetResult_Request__fini(arduinobot_msgs__action__ArduinobotTask_GetResult_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
}

bool
arduinobot_msgs__action__ArduinobotTask_GetResult_Request__are_equal(const arduinobot_msgs__action__ArduinobotTask_GetResult_Request * lhs, const arduinobot_msgs__action__ArduinobotTask_GetResult_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  return true;
}

bool
arduinobot_msgs__action__ArduinobotTask_GetResult_Request__copy(
  const arduinobot_msgs__action__ArduinobotTask_GetResult_Request * input,
  arduinobot_msgs__action__ArduinobotTask_GetResult_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  return true;
}

arduinobot_msgs__action__ArduinobotTask_GetResult_Request *
arduinobot_msgs__action__ArduinobotTask_GetResult_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arduinobot_msgs__action__ArduinobotTask_GetResult_Request * msg = (arduinobot_msgs__action__ArduinobotTask_GetResult_Request *)allocator.allocate(sizeof(arduinobot_msgs__action__ArduinobotTask_GetResult_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(arduinobot_msgs__action__ArduinobotTask_GetResult_Request));
  bool success = arduinobot_msgs__action__ArduinobotTask_GetResult_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
arduinobot_msgs__action__ArduinobotTask_GetResult_Request__destroy(arduinobot_msgs__action__ArduinobotTask_GetResult_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    arduinobot_msgs__action__ArduinobotTask_GetResult_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
arduinobot_msgs__action__ArduinobotTask_GetResult_Request__Sequence__init(arduinobot_msgs__action__ArduinobotTask_GetResult_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arduinobot_msgs__action__ArduinobotTask_GetResult_Request * data = NULL;

  if (size) {
    data = (arduinobot_msgs__action__ArduinobotTask_GetResult_Request *)allocator.zero_allocate(size, sizeof(arduinobot_msgs__action__ArduinobotTask_GetResult_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = arduinobot_msgs__action__ArduinobotTask_GetResult_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        arduinobot_msgs__action__ArduinobotTask_GetResult_Request__fini(&data[i - 1]);
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
arduinobot_msgs__action__ArduinobotTask_GetResult_Request__Sequence__fini(arduinobot_msgs__action__ArduinobotTask_GetResult_Request__Sequence * array)
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
      arduinobot_msgs__action__ArduinobotTask_GetResult_Request__fini(&array->data[i]);
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

arduinobot_msgs__action__ArduinobotTask_GetResult_Request__Sequence *
arduinobot_msgs__action__ArduinobotTask_GetResult_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arduinobot_msgs__action__ArduinobotTask_GetResult_Request__Sequence * array = (arduinobot_msgs__action__ArduinobotTask_GetResult_Request__Sequence *)allocator.allocate(sizeof(arduinobot_msgs__action__ArduinobotTask_GetResult_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = arduinobot_msgs__action__ArduinobotTask_GetResult_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
arduinobot_msgs__action__ArduinobotTask_GetResult_Request__Sequence__destroy(arduinobot_msgs__action__ArduinobotTask_GetResult_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    arduinobot_msgs__action__ArduinobotTask_GetResult_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
arduinobot_msgs__action__ArduinobotTask_GetResult_Request__Sequence__are_equal(const arduinobot_msgs__action__ArduinobotTask_GetResult_Request__Sequence * lhs, const arduinobot_msgs__action__ArduinobotTask_GetResult_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!arduinobot_msgs__action__ArduinobotTask_GetResult_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
arduinobot_msgs__action__ArduinobotTask_GetResult_Request__Sequence__copy(
  const arduinobot_msgs__action__ArduinobotTask_GetResult_Request__Sequence * input,
  arduinobot_msgs__action__ArduinobotTask_GetResult_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(arduinobot_msgs__action__ArduinobotTask_GetResult_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    arduinobot_msgs__action__ArduinobotTask_GetResult_Request * data =
      (arduinobot_msgs__action__ArduinobotTask_GetResult_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!arduinobot_msgs__action__ArduinobotTask_GetResult_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          arduinobot_msgs__action__ArduinobotTask_GetResult_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!arduinobot_msgs__action__ArduinobotTask_GetResult_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `result`
// already included above
// #include "arduinobot_msgs/action/detail/arduinobot_task__functions.h"

bool
arduinobot_msgs__action__ArduinobotTask_GetResult_Response__init(arduinobot_msgs__action__ArduinobotTask_GetResult_Response * msg)
{
  if (!msg) {
    return false;
  }
  // status
  // result
  if (!arduinobot_msgs__action__ArduinobotTask_Result__init(&msg->result)) {
    arduinobot_msgs__action__ArduinobotTask_GetResult_Response__fini(msg);
    return false;
  }
  return true;
}

void
arduinobot_msgs__action__ArduinobotTask_GetResult_Response__fini(arduinobot_msgs__action__ArduinobotTask_GetResult_Response * msg)
{
  if (!msg) {
    return;
  }
  // status
  // result
  arduinobot_msgs__action__ArduinobotTask_Result__fini(&msg->result);
}

bool
arduinobot_msgs__action__ArduinobotTask_GetResult_Response__are_equal(const arduinobot_msgs__action__ArduinobotTask_GetResult_Response * lhs, const arduinobot_msgs__action__ArduinobotTask_GetResult_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  // result
  if (!arduinobot_msgs__action__ArduinobotTask_Result__are_equal(
      &(lhs->result), &(rhs->result)))
  {
    return false;
  }
  return true;
}

bool
arduinobot_msgs__action__ArduinobotTask_GetResult_Response__copy(
  const arduinobot_msgs__action__ArduinobotTask_GetResult_Response * input,
  arduinobot_msgs__action__ArduinobotTask_GetResult_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // status
  output->status = input->status;
  // result
  if (!arduinobot_msgs__action__ArduinobotTask_Result__copy(
      &(input->result), &(output->result)))
  {
    return false;
  }
  return true;
}

arduinobot_msgs__action__ArduinobotTask_GetResult_Response *
arduinobot_msgs__action__ArduinobotTask_GetResult_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arduinobot_msgs__action__ArduinobotTask_GetResult_Response * msg = (arduinobot_msgs__action__ArduinobotTask_GetResult_Response *)allocator.allocate(sizeof(arduinobot_msgs__action__ArduinobotTask_GetResult_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(arduinobot_msgs__action__ArduinobotTask_GetResult_Response));
  bool success = arduinobot_msgs__action__ArduinobotTask_GetResult_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
arduinobot_msgs__action__ArduinobotTask_GetResult_Response__destroy(arduinobot_msgs__action__ArduinobotTask_GetResult_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    arduinobot_msgs__action__ArduinobotTask_GetResult_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
arduinobot_msgs__action__ArduinobotTask_GetResult_Response__Sequence__init(arduinobot_msgs__action__ArduinobotTask_GetResult_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arduinobot_msgs__action__ArduinobotTask_GetResult_Response * data = NULL;

  if (size) {
    data = (arduinobot_msgs__action__ArduinobotTask_GetResult_Response *)allocator.zero_allocate(size, sizeof(arduinobot_msgs__action__ArduinobotTask_GetResult_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = arduinobot_msgs__action__ArduinobotTask_GetResult_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        arduinobot_msgs__action__ArduinobotTask_GetResult_Response__fini(&data[i - 1]);
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
arduinobot_msgs__action__ArduinobotTask_GetResult_Response__Sequence__fini(arduinobot_msgs__action__ArduinobotTask_GetResult_Response__Sequence * array)
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
      arduinobot_msgs__action__ArduinobotTask_GetResult_Response__fini(&array->data[i]);
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

arduinobot_msgs__action__ArduinobotTask_GetResult_Response__Sequence *
arduinobot_msgs__action__ArduinobotTask_GetResult_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arduinobot_msgs__action__ArduinobotTask_GetResult_Response__Sequence * array = (arduinobot_msgs__action__ArduinobotTask_GetResult_Response__Sequence *)allocator.allocate(sizeof(arduinobot_msgs__action__ArduinobotTask_GetResult_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = arduinobot_msgs__action__ArduinobotTask_GetResult_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
arduinobot_msgs__action__ArduinobotTask_GetResult_Response__Sequence__destroy(arduinobot_msgs__action__ArduinobotTask_GetResult_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    arduinobot_msgs__action__ArduinobotTask_GetResult_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
arduinobot_msgs__action__ArduinobotTask_GetResult_Response__Sequence__are_equal(const arduinobot_msgs__action__ArduinobotTask_GetResult_Response__Sequence * lhs, const arduinobot_msgs__action__ArduinobotTask_GetResult_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!arduinobot_msgs__action__ArduinobotTask_GetResult_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
arduinobot_msgs__action__ArduinobotTask_GetResult_Response__Sequence__copy(
  const arduinobot_msgs__action__ArduinobotTask_GetResult_Response__Sequence * input,
  arduinobot_msgs__action__ArduinobotTask_GetResult_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(arduinobot_msgs__action__ArduinobotTask_GetResult_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    arduinobot_msgs__action__ArduinobotTask_GetResult_Response * data =
      (arduinobot_msgs__action__ArduinobotTask_GetResult_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!arduinobot_msgs__action__ArduinobotTask_GetResult_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          arduinobot_msgs__action__ArduinobotTask_GetResult_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!arduinobot_msgs__action__ArduinobotTask_GetResult_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `info`
// already included above
// #include "service_msgs/msg/detail/service_event_info__functions.h"
// Member `request`
// Member `response`
// already included above
// #include "arduinobot_msgs/action/detail/arduinobot_task__functions.h"

bool
arduinobot_msgs__action__ArduinobotTask_GetResult_Event__init(arduinobot_msgs__action__ArduinobotTask_GetResult_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    arduinobot_msgs__action__ArduinobotTask_GetResult_Event__fini(msg);
    return false;
  }
  // request
  if (!arduinobot_msgs__action__ArduinobotTask_GetResult_Request__Sequence__init(&msg->request, 0)) {
    arduinobot_msgs__action__ArduinobotTask_GetResult_Event__fini(msg);
    return false;
  }
  // response
  if (!arduinobot_msgs__action__ArduinobotTask_GetResult_Response__Sequence__init(&msg->response, 0)) {
    arduinobot_msgs__action__ArduinobotTask_GetResult_Event__fini(msg);
    return false;
  }
  return true;
}

void
arduinobot_msgs__action__ArduinobotTask_GetResult_Event__fini(arduinobot_msgs__action__ArduinobotTask_GetResult_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  arduinobot_msgs__action__ArduinobotTask_GetResult_Request__Sequence__fini(&msg->request);
  // response
  arduinobot_msgs__action__ArduinobotTask_GetResult_Response__Sequence__fini(&msg->response);
}

bool
arduinobot_msgs__action__ArduinobotTask_GetResult_Event__are_equal(const arduinobot_msgs__action__ArduinobotTask_GetResult_Event * lhs, const arduinobot_msgs__action__ArduinobotTask_GetResult_Event * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__are_equal(
      &(lhs->info), &(rhs->info)))
  {
    return false;
  }
  // request
  if (!arduinobot_msgs__action__ArduinobotTask_GetResult_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!arduinobot_msgs__action__ArduinobotTask_GetResult_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
arduinobot_msgs__action__ArduinobotTask_GetResult_Event__copy(
  const arduinobot_msgs__action__ArduinobotTask_GetResult_Event * input,
  arduinobot_msgs__action__ArduinobotTask_GetResult_Event * output)
{
  if (!input || !output) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__copy(
      &(input->info), &(output->info)))
  {
    return false;
  }
  // request
  if (!arduinobot_msgs__action__ArduinobotTask_GetResult_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!arduinobot_msgs__action__ArduinobotTask_GetResult_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

arduinobot_msgs__action__ArduinobotTask_GetResult_Event *
arduinobot_msgs__action__ArduinobotTask_GetResult_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arduinobot_msgs__action__ArduinobotTask_GetResult_Event * msg = (arduinobot_msgs__action__ArduinobotTask_GetResult_Event *)allocator.allocate(sizeof(arduinobot_msgs__action__ArduinobotTask_GetResult_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(arduinobot_msgs__action__ArduinobotTask_GetResult_Event));
  bool success = arduinobot_msgs__action__ArduinobotTask_GetResult_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
arduinobot_msgs__action__ArduinobotTask_GetResult_Event__destroy(arduinobot_msgs__action__ArduinobotTask_GetResult_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    arduinobot_msgs__action__ArduinobotTask_GetResult_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
arduinobot_msgs__action__ArduinobotTask_GetResult_Event__Sequence__init(arduinobot_msgs__action__ArduinobotTask_GetResult_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arduinobot_msgs__action__ArduinobotTask_GetResult_Event * data = NULL;

  if (size) {
    data = (arduinobot_msgs__action__ArduinobotTask_GetResult_Event *)allocator.zero_allocate(size, sizeof(arduinobot_msgs__action__ArduinobotTask_GetResult_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = arduinobot_msgs__action__ArduinobotTask_GetResult_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        arduinobot_msgs__action__ArduinobotTask_GetResult_Event__fini(&data[i - 1]);
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
arduinobot_msgs__action__ArduinobotTask_GetResult_Event__Sequence__fini(arduinobot_msgs__action__ArduinobotTask_GetResult_Event__Sequence * array)
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
      arduinobot_msgs__action__ArduinobotTask_GetResult_Event__fini(&array->data[i]);
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

arduinobot_msgs__action__ArduinobotTask_GetResult_Event__Sequence *
arduinobot_msgs__action__ArduinobotTask_GetResult_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arduinobot_msgs__action__ArduinobotTask_GetResult_Event__Sequence * array = (arduinobot_msgs__action__ArduinobotTask_GetResult_Event__Sequence *)allocator.allocate(sizeof(arduinobot_msgs__action__ArduinobotTask_GetResult_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = arduinobot_msgs__action__ArduinobotTask_GetResult_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
arduinobot_msgs__action__ArduinobotTask_GetResult_Event__Sequence__destroy(arduinobot_msgs__action__ArduinobotTask_GetResult_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    arduinobot_msgs__action__ArduinobotTask_GetResult_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
arduinobot_msgs__action__ArduinobotTask_GetResult_Event__Sequence__are_equal(const arduinobot_msgs__action__ArduinobotTask_GetResult_Event__Sequence * lhs, const arduinobot_msgs__action__ArduinobotTask_GetResult_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!arduinobot_msgs__action__ArduinobotTask_GetResult_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
arduinobot_msgs__action__ArduinobotTask_GetResult_Event__Sequence__copy(
  const arduinobot_msgs__action__ArduinobotTask_GetResult_Event__Sequence * input,
  arduinobot_msgs__action__ArduinobotTask_GetResult_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(arduinobot_msgs__action__ArduinobotTask_GetResult_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    arduinobot_msgs__action__ArduinobotTask_GetResult_Event * data =
      (arduinobot_msgs__action__ArduinobotTask_GetResult_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!arduinobot_msgs__action__ArduinobotTask_GetResult_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          arduinobot_msgs__action__ArduinobotTask_GetResult_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!arduinobot_msgs__action__ArduinobotTask_GetResult_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `feedback`
// already included above
// #include "arduinobot_msgs/action/detail/arduinobot_task__functions.h"

bool
arduinobot_msgs__action__ArduinobotTask_FeedbackMessage__init(arduinobot_msgs__action__ArduinobotTask_FeedbackMessage * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    arduinobot_msgs__action__ArduinobotTask_FeedbackMessage__fini(msg);
    return false;
  }
  // feedback
  if (!arduinobot_msgs__action__ArduinobotTask_Feedback__init(&msg->feedback)) {
    arduinobot_msgs__action__ArduinobotTask_FeedbackMessage__fini(msg);
    return false;
  }
  return true;
}

void
arduinobot_msgs__action__ArduinobotTask_FeedbackMessage__fini(arduinobot_msgs__action__ArduinobotTask_FeedbackMessage * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // feedback
  arduinobot_msgs__action__ArduinobotTask_Feedback__fini(&msg->feedback);
}

bool
arduinobot_msgs__action__ArduinobotTask_FeedbackMessage__are_equal(const arduinobot_msgs__action__ArduinobotTask_FeedbackMessage * lhs, const arduinobot_msgs__action__ArduinobotTask_FeedbackMessage * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // feedback
  if (!arduinobot_msgs__action__ArduinobotTask_Feedback__are_equal(
      &(lhs->feedback), &(rhs->feedback)))
  {
    return false;
  }
  return true;
}

bool
arduinobot_msgs__action__ArduinobotTask_FeedbackMessage__copy(
  const arduinobot_msgs__action__ArduinobotTask_FeedbackMessage * input,
  arduinobot_msgs__action__ArduinobotTask_FeedbackMessage * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // feedback
  if (!arduinobot_msgs__action__ArduinobotTask_Feedback__copy(
      &(input->feedback), &(output->feedback)))
  {
    return false;
  }
  return true;
}

arduinobot_msgs__action__ArduinobotTask_FeedbackMessage *
arduinobot_msgs__action__ArduinobotTask_FeedbackMessage__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arduinobot_msgs__action__ArduinobotTask_FeedbackMessage * msg = (arduinobot_msgs__action__ArduinobotTask_FeedbackMessage *)allocator.allocate(sizeof(arduinobot_msgs__action__ArduinobotTask_FeedbackMessage), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(arduinobot_msgs__action__ArduinobotTask_FeedbackMessage));
  bool success = arduinobot_msgs__action__ArduinobotTask_FeedbackMessage__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
arduinobot_msgs__action__ArduinobotTask_FeedbackMessage__destroy(arduinobot_msgs__action__ArduinobotTask_FeedbackMessage * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    arduinobot_msgs__action__ArduinobotTask_FeedbackMessage__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
arduinobot_msgs__action__ArduinobotTask_FeedbackMessage__Sequence__init(arduinobot_msgs__action__ArduinobotTask_FeedbackMessage__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arduinobot_msgs__action__ArduinobotTask_FeedbackMessage * data = NULL;

  if (size) {
    data = (arduinobot_msgs__action__ArduinobotTask_FeedbackMessage *)allocator.zero_allocate(size, sizeof(arduinobot_msgs__action__ArduinobotTask_FeedbackMessage), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = arduinobot_msgs__action__ArduinobotTask_FeedbackMessage__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        arduinobot_msgs__action__ArduinobotTask_FeedbackMessage__fini(&data[i - 1]);
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
arduinobot_msgs__action__ArduinobotTask_FeedbackMessage__Sequence__fini(arduinobot_msgs__action__ArduinobotTask_FeedbackMessage__Sequence * array)
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
      arduinobot_msgs__action__ArduinobotTask_FeedbackMessage__fini(&array->data[i]);
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

arduinobot_msgs__action__ArduinobotTask_FeedbackMessage__Sequence *
arduinobot_msgs__action__ArduinobotTask_FeedbackMessage__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arduinobot_msgs__action__ArduinobotTask_FeedbackMessage__Sequence * array = (arduinobot_msgs__action__ArduinobotTask_FeedbackMessage__Sequence *)allocator.allocate(sizeof(arduinobot_msgs__action__ArduinobotTask_FeedbackMessage__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = arduinobot_msgs__action__ArduinobotTask_FeedbackMessage__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
arduinobot_msgs__action__ArduinobotTask_FeedbackMessage__Sequence__destroy(arduinobot_msgs__action__ArduinobotTask_FeedbackMessage__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    arduinobot_msgs__action__ArduinobotTask_FeedbackMessage__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
arduinobot_msgs__action__ArduinobotTask_FeedbackMessage__Sequence__are_equal(const arduinobot_msgs__action__ArduinobotTask_FeedbackMessage__Sequence * lhs, const arduinobot_msgs__action__ArduinobotTask_FeedbackMessage__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!arduinobot_msgs__action__ArduinobotTask_FeedbackMessage__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
arduinobot_msgs__action__ArduinobotTask_FeedbackMessage__Sequence__copy(
  const arduinobot_msgs__action__ArduinobotTask_FeedbackMessage__Sequence * input,
  arduinobot_msgs__action__ArduinobotTask_FeedbackMessage__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(arduinobot_msgs__action__ArduinobotTask_FeedbackMessage);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    arduinobot_msgs__action__ArduinobotTask_FeedbackMessage * data =
      (arduinobot_msgs__action__ArduinobotTask_FeedbackMessage *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!arduinobot_msgs__action__ArduinobotTask_FeedbackMessage__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          arduinobot_msgs__action__ArduinobotTask_FeedbackMessage__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!arduinobot_msgs__action__ArduinobotTask_FeedbackMessage__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
