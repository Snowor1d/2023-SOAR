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
 * @file OpticalFlow.h
 * This header file contains the declaration of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#ifndef _PX4_MSGS_MSG_OPTICALFLOW_H_
#define _PX4_MSGS_MSG_OPTICALFLOW_H_

// TODO Poner en el contexto.

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
#if defined(OpticalFlow_SOURCE)
#define OpticalFlow_DllAPI __declspec( dllexport )
#else
#define OpticalFlow_DllAPI __declspec( dllimport )
#endif // OpticalFlow_SOURCE
#else
#define OpticalFlow_DllAPI
#endif
#else
#define OpticalFlow_DllAPI
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
        namespace OpticalFlow_Constants
        {
            const uint8_t MODE_UNKNOWN = 0;
            const uint8_t MODE_BRIGHT = 1;
            const uint8_t MODE_LOWLIGHT = 2;
            const uint8_t MODE_SUPER_LOWLIGHT = 3;
        }
        /*!
         * @brief This class represents the structure OpticalFlow defined by the user in the IDL file.
         * @ingroup OPTICALFLOW
         */
        class OpticalFlow
        {
        public:

            /*!
             * @brief Default constructor.
             */
            eProsima_user_DllExport OpticalFlow();

            /*!
             * @brief Default destructor.
             */
            eProsima_user_DllExport ~OpticalFlow();

            /*!
             * @brief Copy constructor.
             * @param x Reference to the object px4_msgs::msg::OpticalFlow that will be copied.
             */
            eProsima_user_DllExport OpticalFlow(const OpticalFlow &x);

            /*!
             * @brief Move constructor.
             * @param x Reference to the object px4_msgs::msg::OpticalFlow that will be copied.
             */
            eProsima_user_DllExport OpticalFlow(OpticalFlow &&x);

            /*!
             * @brief Copy assignment.
             * @param x Reference to the object px4_msgs::msg::OpticalFlow that will be copied.
             */
            eProsima_user_DllExport OpticalFlow& operator=(const OpticalFlow &x);

            /*!
             * @brief Move assignment.
             * @param x Reference to the object px4_msgs::msg::OpticalFlow that will be copied.
             */
            eProsima_user_DllExport OpticalFlow& operator=(OpticalFlow &&x);

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
             * @brief This function sets a value in member sensor_id
             * @param _sensor_id New value for member sensor_id
             */
            eProsima_user_DllExport void sensor_id(uint8_t _sensor_id);

            /*!
             * @brief This function returns the value of member sensor_id
             * @return Value of member sensor_id
             */
            eProsima_user_DllExport uint8_t sensor_id() const;

            /*!
             * @brief This function returns a reference to member sensor_id
             * @return Reference to member sensor_id
             */
            eProsima_user_DllExport uint8_t& sensor_id();

            /*!
             * @brief This function sets a value in member pixel_flow_x_integral
             * @param _pixel_flow_x_integral New value for member pixel_flow_x_integral
             */
            eProsima_user_DllExport void pixel_flow_x_integral(float _pixel_flow_x_integral);

            /*!
             * @brief This function returns the value of member pixel_flow_x_integral
             * @return Value of member pixel_flow_x_integral
             */
            eProsima_user_DllExport float pixel_flow_x_integral() const;

            /*!
             * @brief This function returns a reference to member pixel_flow_x_integral
             * @return Reference to member pixel_flow_x_integral
             */
            eProsima_user_DllExport float& pixel_flow_x_integral();

            /*!
             * @brief This function sets a value in member pixel_flow_y_integral
             * @param _pixel_flow_y_integral New value for member pixel_flow_y_integral
             */
            eProsima_user_DllExport void pixel_flow_y_integral(float _pixel_flow_y_integral);

            /*!
             * @brief This function returns the value of member pixel_flow_y_integral
             * @return Value of member pixel_flow_y_integral
             */
            eProsima_user_DllExport float pixel_flow_y_integral() const;

            /*!
             * @brief This function returns a reference to member pixel_flow_y_integral
             * @return Reference to member pixel_flow_y_integral
             */
            eProsima_user_DllExport float& pixel_flow_y_integral();

            /*!
             * @brief This function sets a value in member gyro_x_rate_integral
             * @param _gyro_x_rate_integral New value for member gyro_x_rate_integral
             */
            eProsima_user_DllExport void gyro_x_rate_integral(float _gyro_x_rate_integral);

            /*!
             * @brief This function returns the value of member gyro_x_rate_integral
             * @return Value of member gyro_x_rate_integral
             */
            eProsima_user_DllExport float gyro_x_rate_integral() const;

            /*!
             * @brief This function returns a reference to member gyro_x_rate_integral
             * @return Reference to member gyro_x_rate_integral
             */
            eProsima_user_DllExport float& gyro_x_rate_integral();

            /*!
             * @brief This function sets a value in member gyro_y_rate_integral
             * @param _gyro_y_rate_integral New value for member gyro_y_rate_integral
             */
            eProsima_user_DllExport void gyro_y_rate_integral(float _gyro_y_rate_integral);

            /*!
             * @brief This function returns the value of member gyro_y_rate_integral
             * @return Value of member gyro_y_rate_integral
             */
            eProsima_user_DllExport float gyro_y_rate_integral() const;

            /*!
             * @brief This function returns a reference to member gyro_y_rate_integral
             * @return Reference to member gyro_y_rate_integral
             */
            eProsima_user_DllExport float& gyro_y_rate_integral();

            /*!
             * @brief This function sets a value in member gyro_z_rate_integral
             * @param _gyro_z_rate_integral New value for member gyro_z_rate_integral
             */
            eProsima_user_DllExport void gyro_z_rate_integral(float _gyro_z_rate_integral);

            /*!
             * @brief This function returns the value of member gyro_z_rate_integral
             * @return Value of member gyro_z_rate_integral
             */
            eProsima_user_DllExport float gyro_z_rate_integral() const;

            /*!
             * @brief This function returns a reference to member gyro_z_rate_integral
             * @return Reference to member gyro_z_rate_integral
             */
            eProsima_user_DllExport float& gyro_z_rate_integral();

            /*!
             * @brief This function sets a value in member ground_distance_m
             * @param _ground_distance_m New value for member ground_distance_m
             */
            eProsima_user_DllExport void ground_distance_m(float _ground_distance_m);

            /*!
             * @brief This function returns the value of member ground_distance_m
             * @return Value of member ground_distance_m
             */
            eProsima_user_DllExport float ground_distance_m() const;

            /*!
             * @brief This function returns a reference to member ground_distance_m
             * @return Reference to member ground_distance_m
             */
            eProsima_user_DllExport float& ground_distance_m();

            /*!
             * @brief This function sets a value in member integration_timespan
             * @param _integration_timespan New value for member integration_timespan
             */
            eProsima_user_DllExport void integration_timespan(uint32_t _integration_timespan);

            /*!
             * @brief This function returns the value of member integration_timespan
             * @return Value of member integration_timespan
             */
            eProsima_user_DllExport uint32_t integration_timespan() const;

            /*!
             * @brief This function returns a reference to member integration_timespan
             * @return Reference to member integration_timespan
             */
            eProsima_user_DllExport uint32_t& integration_timespan();

            /*!
             * @brief This function sets a value in member time_since_last_sonar_update
             * @param _time_since_last_sonar_update New value for member time_since_last_sonar_update
             */
            eProsima_user_DllExport void time_since_last_sonar_update(uint32_t _time_since_last_sonar_update);

            /*!
             * @brief This function returns the value of member time_since_last_sonar_update
             * @return Value of member time_since_last_sonar_update
             */
            eProsima_user_DllExport uint32_t time_since_last_sonar_update() const;

            /*!
             * @brief This function returns a reference to member time_since_last_sonar_update
             * @return Reference to member time_since_last_sonar_update
             */
            eProsima_user_DllExport uint32_t& time_since_last_sonar_update();

            /*!
             * @brief This function sets a value in member frame_count_since_last_readout
             * @param _frame_count_since_last_readout New value for member frame_count_since_last_readout
             */
            eProsima_user_DllExport void frame_count_since_last_readout(uint16_t _frame_count_since_last_readout);

            /*!
             * @brief This function returns the value of member frame_count_since_last_readout
             * @return Value of member frame_count_since_last_readout
             */
            eProsima_user_DllExport uint16_t frame_count_since_last_readout() const;

            /*!
             * @brief This function returns a reference to member frame_count_since_last_readout
             * @return Reference to member frame_count_since_last_readout
             */
            eProsima_user_DllExport uint16_t& frame_count_since_last_readout();

            /*!
             * @brief This function sets a value in member gyro_temperature
             * @param _gyro_temperature New value for member gyro_temperature
             */
            eProsima_user_DllExport void gyro_temperature(int16_t _gyro_temperature);

            /*!
             * @brief This function returns the value of member gyro_temperature
             * @return Value of member gyro_temperature
             */
            eProsima_user_DllExport int16_t gyro_temperature() const;

            /*!
             * @brief This function returns a reference to member gyro_temperature
             * @return Reference to member gyro_temperature
             */
            eProsima_user_DllExport int16_t& gyro_temperature();

            /*!
             * @brief This function sets a value in member quality
             * @param _quality New value for member quality
             */
            eProsima_user_DllExport void quality(uint8_t _quality);

            /*!
             * @brief This function returns the value of member quality
             * @return Value of member quality
             */
            eProsima_user_DllExport uint8_t quality() const;

            /*!
             * @brief This function returns a reference to member quality
             * @return Reference to member quality
             */
            eProsima_user_DllExport uint8_t& quality();

            /*!
             * @brief This function sets a value in member max_flow_rate
             * @param _max_flow_rate New value for member max_flow_rate
             */
            eProsima_user_DllExport void max_flow_rate(float _max_flow_rate);

            /*!
             * @brief This function returns the value of member max_flow_rate
             * @return Value of member max_flow_rate
             */
            eProsima_user_DllExport float max_flow_rate() const;

            /*!
             * @brief This function returns a reference to member max_flow_rate
             * @return Reference to member max_flow_rate
             */
            eProsima_user_DllExport float& max_flow_rate();

            /*!
             * @brief This function sets a value in member min_ground_distance
             * @param _min_ground_distance New value for member min_ground_distance
             */
            eProsima_user_DllExport void min_ground_distance(float _min_ground_distance);

            /*!
             * @brief This function returns the value of member min_ground_distance
             * @return Value of member min_ground_distance
             */
            eProsima_user_DllExport float min_ground_distance() const;

            /*!
             * @brief This function returns a reference to member min_ground_distance
             * @return Reference to member min_ground_distance
             */
            eProsima_user_DllExport float& min_ground_distance();

            /*!
             * @brief This function sets a value in member max_ground_distance
             * @param _max_ground_distance New value for member max_ground_distance
             */
            eProsima_user_DllExport void max_ground_distance(float _max_ground_distance);

            /*!
             * @brief This function returns the value of member max_ground_distance
             * @return Value of member max_ground_distance
             */
            eProsima_user_DllExport float max_ground_distance() const;

            /*!
             * @brief This function returns a reference to member max_ground_distance
             * @return Reference to member max_ground_distance
             */
            eProsima_user_DllExport float& max_ground_distance();

            /*!
             * @brief This function sets a value in member mode
             * @param _mode New value for member mode
             */
            eProsima_user_DllExport void mode(uint8_t _mode);

            /*!
             * @brief This function returns the value of member mode
             * @return Value of member mode
             */
            eProsima_user_DllExport uint8_t mode() const;

            /*!
             * @brief This function returns a reference to member mode
             * @return Reference to member mode
             */
            eProsima_user_DllExport uint8_t& mode();


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
            eProsima_user_DllExport static size_t getCdrSerializedSize(const px4_msgs::msg::OpticalFlow& data, size_t current_alignment = 0);


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
            uint8_t m_sensor_id;
            float m_pixel_flow_x_integral;
            float m_pixel_flow_y_integral;
            float m_gyro_x_rate_integral;
            float m_gyro_y_rate_integral;
            float m_gyro_z_rate_integral;
            float m_ground_distance_m;
            uint32_t m_integration_timespan;
            uint32_t m_time_since_last_sonar_update;
            uint16_t m_frame_count_since_last_readout;
            int16_t m_gyro_temperature;
            uint8_t m_quality;
            float m_max_flow_rate;
            float m_min_ground_distance;
            float m_max_ground_distance;
            uint8_t m_mode;
        };
    }
}

#endif // _PX4_MSGS_MSG_OPTICALFLOW_H_