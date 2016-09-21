/*
 * XmlRpcConversion.hpp
 *
 *  Created on: Oct 20, 2011
 *      Author: mriedel
 */

#ifndef XMLRPCCONVERSION_HPP_
#define XMLRPCCONVERSION_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include <telekyb_defines/enum.hpp>

//XmlRPC
#include <XmlRpc.h>

// ros
#include <ros/console.h>

// Eigen
#include <Eigen/Dense>

// Implementation of StringCoversion Classes
namespace TELEKYB_NAMESPACE
{
// These all throw XmlRpc::XmlRpcException!

template < class _T >
void operator >> (const _T& input, XmlRpc::XmlRpcValue& output)
{
	output = XmlRpc::XmlRpcValue(input);
}

template < class _T >
void operator >> (XmlRpc::XmlRpcValue input, _T& output)
{
	output = static_cast<_T>(input);
}

// Eigen::Matrix
template<typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
void operator >> (const Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& matrix, XmlRpc::XmlRpcValue& value)
{
	int nValues = matrix.rows() * matrix.cols();
	value.setSize(nValues);

	for (int i = 0; i < nValues; ++i) {
		value[i] = XmlRpc::XmlRpcValue( matrix(i / matrix.cols(), i % matrix.cols() ) );
	}

}

template<typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
void operator >> (XmlRpc::XmlRpcValue value, Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& matrix)
{
	int nValues = value.size();

	if (nValues == matrix.rows() * matrix.cols() ) {
		// parse string into Matrix
		for (int i = 0; i < matrix.rows(); i++) {
			for (int j = 0; j < matrix.cols(); j++) {
				matrix(i, j) = static_cast<_Scalar>(value[i*matrix.cols() + j]);
			}
		}
	} else {
		std::stringstream sstream;
		sstream << "Unable to fill Eigen::Matrix. Input is of wrong size! Matrix(" <<
				matrix.rows() <<"x"<< matrix.cols() <<") != XmlRpcValue.size("<< nValues <<")";
		throw XmlRpc::XmlRpcException(sstream.str());
	}
}

template<typename _Scalar, int _Options>
void operator >> (const Eigen::Quaternion<_Scalar,_Options>& input, XmlRpc::XmlRpcValue& value)
{
	value.setSize(4); // Quaterion has 4 Elements

	value[0] = XmlRpc::XmlRpcValue( input.w() );
	value[1] = XmlRpc::XmlRpcValue( input.x() );
	value[2] = XmlRpc::XmlRpcValue( input.y() );
	value[3] = XmlRpc::XmlRpcValue( input.z() );
}

template<typename _Scalar, int _Options>
void operator >> (XmlRpc::XmlRpcValue input, Eigen::Quaternion<_Scalar,_Options>& quaternion)
{
	if (input.size() != 4) {
		std::stringstream sstream;
		sstream << "Unable to fill Eigen::QuaternionBase< Eigen::Quaternion<_Scalar,_Options> >. "
				"Input is of wrong size! XmlRpcValue.size(" <<
				input.size() <<") != 4)";
		throw XmlRpc::XmlRpcException(sstream.str());
	}

	// size is ok
	quaternion.w() = static_cast<_Scalar>(input[0]);
	quaternion.x() = static_cast<_Scalar>(input[1]);
	quaternion.y() = static_cast<_Scalar>(input[2]);
	quaternion.z() = static_cast<_Scalar>(input[3]);

}

// std::vector
template<class _T>
void operator >> (const std::vector<_T>& input, XmlRpc::XmlRpcValue& output)
{
	output.setSize(input.size());
	for (size_t i = 0; i < input.size(); i++) {
 		input.at(i) >> output[i];
	}
}

template<class _T>
void operator >> (XmlRpc::XmlRpcValue input, std::vector<_T>& output)
{
	if (input.size() == (signed)output.size()) {
		for (size_t i = 0; i < output.size(); i++) {
			input[i] >> output[i];
		}
	} else {
		std::stringstream sstream;
		sstream << "Unable to fill boost::array. Input is of wrong size! std::vector(" <<
				output.size() <<") != XmlRpcValue.size("<< input.size() <<")";
		throw XmlRpc::XmlRpcException(sstream.str());
	}
	//output = static_cast<_T>(input);
}

// Boost Array
template<class _T, std::size_t _N>
void operator >> (const boost::array<_T,_N>& input, XmlRpc::XmlRpcValue& output)
{
	output.setSize(_N);
	for (size_t i = 0; i < _N; i++) {
 		input.at(i) >> output[i];
	}
}

template<class _T, std::size_t _N>
void operator >> (XmlRpc::XmlRpcValue input, boost::array<_T,_N>& output)
{
	if (input.size() == _N) {
		for (size_t i = 0; i < _N; i++) {
			input[i] >> output[i];
		}
	} else {
		std::stringstream sstream;
		sstream << "Unable to fill boost::array. Input is of wrong size! boost::array(" <<
				_N <<") != XmlRpcValue.size("<< input.size() <<")";
		throw XmlRpc::XmlRpcException(sstream.str());
	}
	//output = static_cast<_T>(input);
}

// Enum
template < class ENUM_, class TYPE_ >
void operator >> (const boost::detail::enum_base<ENUM_, TYPE_>& input, XmlRpc::XmlRpcValue& output)
{
	output = XmlRpc::XmlRpcValue( input.str() );
}

template < class ENUM_, class TYPE_ >
void operator >> (XmlRpc::XmlRpcValue input, boost::detail::enum_base<ENUM_, TYPE_>& output)
{
	std::string enumName = static_cast<std::string>(input);
	boost::optional<ENUM_> result = ENUM_::get_by_name(enumName.c_str());
	if (result) {
		output = *result;
	} else {
		std::string msg = std::string("Unknown enum ") + ENUM_::getEnumName() + "::" + enumName + "! Possible Values: " + ENUM_::getAllDomainNames(',') + ".";
		throw XmlRpc::XmlRpcException(msg);
	}
}

}


#endif /* XMLRPCCONVERSION_HPP_ */
