/*
 * Conversion.h
 *
 *  Created on: Oct 13, 2011
 *      Author: mriedel
 */

#ifndef YAMLCONVERSION_HPP_
#define YAMLCONVERSION_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include <telekyb_defines/enum.hpp>

// stl
#include <string>
#include <vector>

// ros
#include <ros/console.h>

// boost
#include <boost/array.hpp>

// Eigen
#include <Eigen/Dense>

// Yaml
#include <yaml-cpp/yaml.h>



// New Structure
namespace YAML {

// Enum
template<class ENUM_, class TYPE_>
struct convert<boost::detail::enum_base<ENUM_, TYPE_> > {
	static Node encode(const boost::detail::enum_base<ENUM_, TYPE_>& rhs) {
		Node node( rhs.str() );
		return node;
	}

	static bool decode(const Node& node, boost::detail::enum_base<ENUM_, TYPE_>& rhs) {
		std::string strValue = node.as<std::string>();

		boost::optional<ENUM_> result = ENUM_::get_by_name(strValue.c_str());
		if (result) {
			rhs = *result;
		} else {
			ROS_ERROR_STREAM("Unknown enum " << ENUM_::getEnumName() << "::"
					<< strValue << "! Possible Values: "
					<< ENUM_::getAllDomainNames(',') << ".");
			return false;
		}
		return true;
	}
};


template < class _T, std::size_t _N>
struct convert< boost::array<_T, _N> > {
	static Node encode(const boost::array<_T, _N>& rhs) {
		Node node(NodeType::Sequence);
		for (size_t i = 0; i < _N; i++) {
			node.push_back(rhs[i]);
		}
		return node;
	}

	static bool decode(const Node& node, boost::array<_T, _N>& rhs) {
		if (node.size() != _N) {
			ROS_ERROR_STREAM("Unable to create boost::array. Input is of wrong size! boost::array("<<
					_N <<") != Tokens("<< node.size() << ")");
			return false;
		}
		for (size_t i = 0; i < _N; i++) {
			rhs[i] = node[i].as<_T>();
			//node[i] >> value[i];
		}
		return true;
	}
};

template < typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
struct convert< Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> > {
	static Node encode(const Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& matrix) {
		Node node(NodeType::Sequence);

		int nValues = matrix.rows() * matrix.cols();

		for (int i = 0; i < nValues; ++i) {
			node.push_back( matrix(i / matrix.cols(), i % matrix.cols() ) );
		}

		return node;
	}

	static bool decode(const Node& node, Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& matrix) {

		// Totally Dynamic sized arrays are not supported
		if (_Rows == Eigen::Dynamic && _Cols == Eigen::Dynamic) {
			ROS_ERROR("Double Dynamic Matrices are not supported! Matrices may only be dynamic in one dimension!");
			return false;
		}

		int nSize = node.size(); // Sequence check is implicit

		// If one dimension is dynamic -> calculate and resize

		// Rows Dynamic
		if (_Rows == Eigen::Dynamic) {
			// Check
			if (nSize % _Cols != 0) {
				ROS_ERROR("Inputsize (%d) of dynamic row matrix is not a multiple of fixed column size (%d)", nSize, _Cols);
				return false;
			}

			int nDynRows = nSize / _Cols;
			matrix.resize(nDynRows, Eigen::NoChange);
		}

		// Cols Dynamic
		if (_Cols == Eigen::Dynamic) {
			// Check
			if (nSize % _Rows != 0) {
				ROS_ERROR("Inputsize (%d) of dynamic column matrix is not a multiple of fixed row size (%d)", nSize, _Rows);
				return false;
			}

			int nDynCols = nSize / _Rows;
			matrix.resize(Eigen::NoChange, nDynCols);
		}


		if (nSize != matrix.rows() * matrix.cols()) {
			ROS_ERROR_STREAM("Unable to create Eigen::Matrix. Input is of wrong size! Matrix("<<
					matrix.rows() <<"x"<< matrix.cols() <<") != Tokens("<< nSize <<")");
			return false;
		} else {
			// fill
			for (int i = 0; i < matrix.rows(); i++) {
				for (int j = 0; j < matrix.cols(); j++) {
					//ROS_INFO_STREAM("Node pos: " << i*matrix.rows() + j << "Matrix: " << i << "," << j);
//					YAML::Node field = node[i + j*matrix.cols()];
//					matrix(i,j) = field.as<_Scalar>();

					matrix(i,j) = node[(int)(i*matrix.cols() + j)].as<_Scalar>();
				}
			}
		}
		return true;
	}
};



template<typename _Scalar, int _Options>
struct convert< Eigen::Quaternion<_Scalar,_Options> > {
	static Node encode(const Eigen::Quaternion<_Scalar,_Options>& quaternion) {
		Node node(NodeType::Sequence);

		node[0] = quaternion.w();
		node[1] = quaternion.x();
		node[2] = quaternion.y();
		node[3] = quaternion.z();

		return node;
	}

	static bool decode(const Node& node, Eigen::Quaternion<_Scalar,_Options>& quaternion) {

		int nSize = node.size(); // Sequence check is implicit
		if (nSize != 4) {
			ROS_ERROR_STREAM("Unable to create Eigen::QuaternionBase< Eigen::Quaternion<_Scalar,_Options> >. Input is of wrong size! Matrix("<<
					nSize <<") != 4");
			return false;
		} else {
			// fill
			quaternion.w() = node[0].as<_Scalar>();
			quaternion.x() = node[1].as<_Scalar>();
			quaternion.y() = node[2].as<_Scalar>();
			quaternion.z() = node[3].as<_Scalar>();
		}
		return true;
	}
};


}


// Implementation of StringCoversion Classes
namespace TELEKYB_NAMESPACE
{

// ENUM
//template < class ENUM_, class TYPE_ >
//void operator >> (const YAML::Node& node, boost::detail::enum_base<ENUM_, TYPE_>& value)
//{
//	std::string strValue = node.as<std::string>();
//	//node >> strValue;
//	boost::optional<ENUM_> result = ENUM_::get_by_name(strValue.c_str());
//	if (result) {
//		value = *result;
//	} else {
//		std::stringstream sstream;
//		sstream << "Unknown enum " << ENUM_::getEnumName() << "::" << strValue <<
//				"! Possible Values: "<< ENUM_::getAllDomainNames(',') << ".";
//		throw YAML::Exception(YAML::Mark::null(), sstream.str());
//	}
//}

//template < class ENUM_, class TYPE_ >
//void operator >> (const boost::detail::enum_base<ENUM_, TYPE_>& value, YAML::Node& node)
//{
//	value.str() >> node;
//}

// boost array
//template < class _T, std::size_t _N>
//void operator >> (const YAML::Node& node, boost::array<_T, _N>& value)
//{
//	if (node.size() != _N) {
//		std::stringstream sstream;
//		sstream << "Unable to create boost::array. Input is of wrong size! boost::array("<<
//				_N <<") != Tokens("<< node.size() << ")";
//		throw YAML::Exception(YAML::Mark::null(), sstream.str());
//	}
//	for (size_t i = 0; i < _N; i++) {
//		value[i] = node.as<_T>();
//		//node[i] >> value[i];
//	}
//}

//template < class _T, std::size_t _N>
//void operator >> (const boost::array<_T, _N>& value, YAML::Node& node)
//{
//	for (size_t i = 0; i < _N; i++) {
//		value.at(i) >> node[i];
//	}
//}

// Eigen::Matrix
//template < typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
//void operator >> (const YAML::Node& node, Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& matrix)
//{
//	//ROS_INFO_STREAM("Entered YAML Conversion Matrix....");
//	int nSize = node.size();
//	if (nSize != matrix.rows() * matrix.cols()) {
//		std::stringstream sstream;
//		sstream << "Unable to create Eigen::Matrix. Input is of wrong size! Matrix("<<
//				matrix.rows() <<"x"<< matrix.cols() <<") != Tokens("<< nSize <<")";
//				throw YAML::Exception(YAML::Mark::null(), sstream.str());
//	} else {
//		// fill
//		for (int i = 0; i < matrix.rows(); i++) {
//			for (int j = 0; j < matrix.cols(); j++) {
//				//ROS_INFO_STREAM("Node pos: " << i*matrix.rows() + j << "Matrix: " << i << "," << j);
//				//node[i + j*matrix.cols()] >> matrix(i, j);
//				//matrix(i,j) = node[i + j*matrix.cols()].as<_Scalar>();
//			}
//		}
//	}
//}

//template < class ENUM_, class TYPE_ >
//void operator >> (const boost::detail::enum_base<ENUM_, TYPE_>& value, YAML::Node& node)
//{
//
//}
//
//
//template < class _T, std::size_t _N>
//void operator >> (const boost::array<_T, _N>& value, YAML::Node& node)
//{
//	//todo
//}
//
//// Inverse functions
//template < typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
//void operator >> (const Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& matrix, YAML::Node& node)
//{
//	// todo
//}



} // namespace

#endif /* CONVERSION_H_ */
