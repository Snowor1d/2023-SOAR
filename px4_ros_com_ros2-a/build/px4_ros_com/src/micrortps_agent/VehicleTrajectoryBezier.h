// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*!
 * @file VehicleTrajectoryBezier.h
 * This header file contains the declaration of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#ifndef _PX4_MSGS_MSG_VEHICLETRAJECTORYBEZIER_H_
#define _PX4_MSGS_MSG_VEHICLETRAJECTORYBEZIER_H_

// TODO Poner en el contexto.
#include "TrajectoryBezier.h"

#include <stdint.h>
#include <array>
#include <string>
#include <vector>
#include <map>
#include <bitset>

#if defined(_WIN32)
#if defined(EPROSIMA_USER_DLL_EXPORT)
#define eProsima_user_DllExport __declspec( dllexport )
#else
#define eProsima_user_DllExport
#endif
#else
#define eProsima_user_DllExport
#endif

#if defined(_WIN32)
#if defined(EPROSIMA_USER_DLL_EXPORT)
#if defined(VehicleTrajectoryBezier_SOURCE)
#define VehicleTrajectoryBezier_DllAPI __declspec( dllexport )
#else
#define VehicleTrajectoryBezier_DllAPI __declspec( dllimport )
#endif // VehicleTrajectoryBezier_SOURCE
#else
#define VehicleTrajectoryBezier_DllAPI
#endif
#else
#define VehicleTrajectoryBezier_DllAPI
#endif // _WIN32

namespace eprosima
{
    namespace fastcdr
    {
        class Cdr;
    }
}


namespace px4_msgs
{
    namespace msg
    {
        typedef px4_msgs::msg::TrajectoryBezier px4_msgs__msg__TrajectoryBezier;
        typedef std::array<px4_msgs::msg::px4_msgs__msg__TrajectoryBezier, 5> px4_msgs__msg__TrajectoryBezier__5;
        namespace VehicleTrajectoryBezier_Constants
        {
            const uint8_t POINT_0 = 0;
            const uint8_t POINT_1 = 1;
            const uint8_t POINT_2 = 2;
            const uint8_t POINT_3 = 3;
            const uint8_t POINT_4 = 4;
            const uint8_t NUMBER_POINTS = 5;
        }
        /*!
         * @brief This class represents the structure VehicleTrajectoryBezier defined by the user in the IDL file.
         * @ingroup VEHICLETRAJECTORYBEZIER
         */
        class VehicleTrajectoryBezier
        {
        public:

            /*!
             * @brief Default constructor.
             */
            eProsima_user_DllExport VehicleTrajectoryBezier();

            /*!
             * @brief Default destructor.
             */
            eProsima_user_DllExport ~VehicleTrajectoryBezier();

            /*!
             * @brief Copy constructor.
             * @param x Reference to the object px4_msgs::msg::VehicleTrajectoryBezier that will be copied.
             */
            eProsima_user_DllExport VehicleTrajectoryBezier(const VehicleTrajectoryBezier &x);

            /*!
             * @brief Move constructor.
             * @param x Reference to the object px4_msgs::msg::VehicleTrajectoryBezier that will be copied.
             */
            eProsima_user_DllExport VehicleTrajectoryBezier(VehicleTrajectoryBezier &&x);

            /*!
             * @brief Copy assignment.
             * @param x Reference to the object px4_msgs::msg::VehicleTrajectoryBezier that will be copied.
             */
            eProsima_user_DllExport VehicleTrajectoryBezier& operator=(const VehicleTrajectoryBezier &x);

            /*!
             * @brief Move assignment.
             * @param x Reference to the object px4_msgs::msg::VehicleTrajectoryBezier that will be copied.
             */
            eProsima_user_DllExport VehicleTrajectoryBezier& operator=(VehicleTrajectoryBezier &&x);

            /*!
             * @brief This function sets a value in member timestamp
             * @param _timestamp New value for member timestamp
             */
            eProsima_user_DllExport void timestamp(uint64_t _timestamp);

            /*!
             * @brief This function returns the value of member timestamp
             * @return Value of member timestamp
             */
            eProsima_user_DllExport uint64_t timestamp() const;

            /*!
             * @brief This function returns a reference to member timestamp
             * @return Reference to member timestamp
             */
            eProsima_user_DllExport uint64_t& timestamp();

            /*!
             * @brief This function copies the value in member control_points
             * @param _control_points New value to be copied in member control_points
             */
            eProsima_user_DllExport void control_points(const px4_msgs::msg::px4_msgs__msg__TrajectoryBezier__5 &_control_points);

            /*!
             * @brief This function moves the value in member control_points
             * @param _control_points New value to be moved in member control_points
             */
            eProsima_user_DllExport void control_points(px4_msgs::msg::px4_msgs__msg__TrajectoryBezier__5 &&_control_points);

            /*!
             * @brief This function returns a constant reference to member control_points
             * @return Constant reference to member control_points
             */
            eProsima_user_DllExport const px4_msgs::msg::px4_msgs__msg__TrajectoryBezier__5& control_points() const;

            /*!
             * @brief This function returns a reference to member control_points
             * @return Reference to member control_points
             */
            eProsima_user_DllExport px4_msgs::msg::px4_msgs__msg__TrajectoryBezier__5& control_points();
            /*!
             * @brief This function sets a value in member bezier_order
             * @param _bezier_order New value for member bezier_order
             */
            eProsima_user_DllExport void bezier_order(uint8_t _bezier_order);

            /*!
             * @brief This function returns the value of member bezier_order
             * @return Value of member bezier_order
             */
            eProsima_user_DllExport uint8_t bezier_order() const;

            /*!
             * @brief This function returns a reference to member bezier_order
             * @return Reference to member bezier_order
             */
            eProsima_user_DllExport uint8_t& bezier_order();


            /*!
             * @brief This function returns the maximum serialized size of an object
             * depending on the buffer alignment.
             * @param current_alignment Buffer alignment.
             * @return Maximum serialized size.
             */
            eProsima_user_DllExport static size_t getMaxCdrSerializedSize(size_t current_alignment = 0);

            /*!
             * @brief This function returns the serialized size of a data depending on the buffer alignment.
             * @param data Data which is calculated its serialized size.
             * @param current_alignment Buffer alignment.
             * @return Serialized size.
             */
            eProsima_user_DllExport static size_t getCdrSerializedSize(const px4_msgs::msg::VehicleTrajectoryBezier& data, size_t current_alignment = 0);


            /*!
             * @brief This function serializes an object using CDR serialization.
             * @param cdr CDR serialization object.
             */
            eProsima_user_DllExport void serialize(eprosima::fastcdr::Cdr &cdr) const;

            /*!
             * @brief This function deserializes an object using CDR serialization.
             * @param cdr CDR serialization object.
             */
            eProsima_user_DllExport void deserialize(eprosima::fastcdr::Cdr &cdr);



            /*!
             * @brief This function returns the maximum serialized size of the Key of an object
             * depending on the buffer alignment.
             * @param current_alignment Buffer alignment.
             * @return Maximum serialized size.
             */
            eProsima_user_DllExport static size_t getKeyMaxCdrSerializedSize(size_t current_alignment = 0);

            /*!
             * @brief This function tells you if the Key has been defined for this type
             */
            eProsima_user_DllExport static bool isKeyDefined();

            /*!
             * @brief This function serializes the key members of an object using CDR serialization.
             * @param cdr CDR serialization object.
             */
            eProsima_user_DllExport void serializeKey(eprosima::fastcdr::Cdr &cdr) const;

        private:
            uint64_t m_timestamp;
            px4_msgs::msg::px4_msgs__msg__TrajectoryBezier__5 m_control_points;
            uint8_t m_bezier_order;
        };
    }
}

#endif // _PX4_MSGS_MSG_VEHICLETRAJECTORYBEZIER_H_