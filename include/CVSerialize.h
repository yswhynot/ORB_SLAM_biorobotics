#ifndef CVMAT_H_
#define CVMAT_H_

#include <opencv2/opencv.hpp>

#include <boost/serialization/split_free.hpp>
#include <boost/serialization/vector.hpp>

#include <boost/archive/tmpdir.hpp>

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/assume_abstract.hpp>
#include <boost/serialization/version.hpp>
#include <boost/serialization/split_member.hpp>

#include "KeyFrame.h"
#include "Frame.h"
#include "Map.h"
#include "MapPoint.h"
#include "KeyFrameDatabase.h"

BOOST_SERIALIZATION_SPLIT_FREE(::cv::Mat);
namespace boost {
	namespace serialization {

    /** Serialization support for cv::Mat */
    template<class Archive>
		void save(Archive & ar, const ::cv::Mat& m, const unsigned int version) {
			size_t elem_size = m.elemSize();
			size_t elem_type = m.type();

			ar & m.cols;
			ar & m.rows;
			ar & elem_size;
			ar & elem_type;

			const size_t data_size = m.cols * m.rows * elem_size;
			ar & boost::serialization::make_array(m.ptr(), data_size);
		}

    /** Serialization support for cv::Mat */
    template<class Archive>
		void load(Archive & ar, ::cv::Mat& m, const unsigned int version)
		{
			int cols, rows;
			size_t elem_size, elem_type;

			ar & cols;
			ar & rows;
			ar & elem_size;
			ar & elem_type;

			m.create(rows, cols, elem_type);

			size_t data_size = m.cols * m.rows * elem_size;
			ar & boost::serialization::make_array(m.ptr(), data_size);
		}

	}
}

BOOST_SERIALIZATION_SPLIT_FREE(::cv::KeyPoint);
namespace boost {
	namespace serialization {
		template<class Archive>
		void save(Archive& ar, const ::cv::KeyPoint& p, const unsigned int version) {
			ar & p.pt.x;
			ar & p.pt.y;
			ar & p.size;
			ar & p.angle;
			ar & p.response;
			ar & p.octave;
			ar & p.class_id;
		}

		template<class Archive>
		void load(Archive &ar, ::cv::KeyPoint& p, const unsigned int version) {
			ar & p.pt.x;
			ar & p.pt.y;
			ar & p.size;
			ar & p.angle;
			ar & p.response;
			ar & p.octave;
			ar & p.class_id;
		}
	}
}

// namespace boost {
// 	namespace serialization {
// 		// MapPoint constructor serialization
// 		template<class Archive>
// 		inline void save_construct_data(Archive & ar, const MapPoint* mp, const unsigned int file_version) {
// 		    // save data required to construct instance
// 			ar << mp->mWorldPos;
// 			ar << mp->mpRefKF;
// 			ar << mp->mpMap;
// 		}

// 		template<class Archive>
// 		inline void load_construct_data(Archive & ar, MapPoint* mp, const unsigned int file_version ) {
// 		    // retrieve data from archive required to construct new instance
// 			cv::Mat mWorldPos;
// 			KeyFrame* mpRefKF;
// 			Map* mpMap;

// 			ar >> mWorldPos;
// 			ar >> mpRefKF;
// 			ar >> mpMap;
// 		    // invoke inplace constructor to initialize instance of my_class
// 			::new(mp)MapPoint(mWorldPos, mpRefKF, mpMap);
// 		}

// 		// KeyFrame constructor serialization
// 		template<class Archive>
// 		inline void save_construct_data(Archive& ar, const KeyFrame* kf, const unsigned int version) {
// 			ar << kf->mF;
// 			ar << kf->mpMap;
// 			ar << kf->mpKeyFrameDB;
// 		}

// 		template<class Archive>
// 		inline void load_construct_data(Archive& ar, KeyFrame* kf, const unsigned int version) {
// 			Frame mF;
// 			Map* mpMap;
// 			KeyFrameDatabase* mpKeyFrameDB;

// 			ar >> mF;
// 			ar >> mpMap;
// 			ar >> mpKeyFrameDB;

// 			::new(kf)KeyFrame(mF, mpMap, mpKeyFrameDB);
// 		}
// 	}
// }

#endif /* CVMAT_SERIALIZE_HPP_ */
