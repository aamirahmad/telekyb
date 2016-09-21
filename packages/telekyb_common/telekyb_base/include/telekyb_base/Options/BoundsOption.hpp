/*
 * BoundsOption.hpp
 *
 *  Created on: Oct 11, 2011
 *      Author: mriedel
 */

#ifndef BOUNDSOPTION_HPP_
#define BOUNDSOPTION_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include <telekyb_base/Options/Option.hpp>


// ros
#include <ros/console.h>

namespace TELEKYB_NAMESPACE
{

template < class _T, class _Compare = std::less<_T> >
class BoundsOption : public Option<_T> {
protected:
    _T lowerBound;
    _T upperBound;
    _Compare comp;

    BoundsOption(OptionContainer* parent_, const std::string name_, const std::string description_, const _T& defaultValue_, const _T& lowerBound_, const _T& upperBound_,
                 bool mandatory_ = false, bool readOnly_ = false)
        : Option<_T>(parent_, name_, description_, defaultValue_, mandatory_, readOnly_), lowerBound(lowerBound_), upperBound(upperBound_) {
        // Inverted?
        if (!comp(lowerBound_, upperBound_)) {
            ROS_WARN_STREAM("LowerBound > UpperBound! Inverting... Option: " << name_);
            lowerBound = upperBound_;
            upperBound = lowerBound_;
        }

        if (!isWithinBounds(Option<_T>::value)) {
            // outside of bounds
            ROS_ERROR_STREAM("Option " << BaseOption::getNSName() << " out of Bounds!");
            ROS_ERROR_STREAM("Lower Bound: " << YamlHelper::parseValueToString(lowerBound));
            ROS_ERROR_STREAM("Upper Bound: " << YamlHelper::parseValueToString(upperBound));
            if (mandatory_) {
                // fatal
                ROS_FATAL_STREAM("Initial Value ("<< Option<_T>::value <<") of Mandatory(!) Option " << BaseOption::getNSName() << " out of Bounds!");
                //ROS_BREAK();
                ros::shutdown();
            } else {
                ROS_ERROR_STREAM("Initial Value ("<< Option<_T>::value <<") of "
                                 << BaseOption::getNSName() << " is out of Bounds. Resetting to Default: " << defaultValue_);
                Option<_T>::value = defaultValue_;
                Option<_T>::initialValue = true;
            }
        }

    }

    friend class OptionContainer;
public:
    virtual ~BoundsOption(){};

    virtual bool setValueCheck(const _T& value_) {

        if ( !isWithinBounds(value_) ) {
            ROS_WARN_STREAM("Trying to set Value out of Bounds. Value: " << YamlHelper::parseValueToString(value_));
            ROS_WARN_STREAM("Lower Bound: " << YamlHelper::parseValueToString(lowerBound));
            ROS_WARN_STREAM("Upper Bound: " << YamlHelper::parseValueToString(upperBound));
            return false;
        }

        return Option<_T>::setValueCheck(value_);
    }

    //	virtual void setValue(const _T& value_) {
    //
    //		Option<_T>::setValue(value_);
    //	}

    virtual bool hasBounds() const {
        return true;
    }

    virtual bool isWithinBounds(const _T& value_) const {
        return !(comp(value_,lowerBound) || comp(upperBound, value_));
    }

    virtual void print() const {
        std::cout << "--" << BaseOption::getNSName() << " (";
        std::cout << YamlHelper::parseValueToString( Option<_T>::getValue() ) << ", ";
        std::cout << "man: " << YamlHelper::parseValueToString( Option<_T>::mandatory ) << ", ";
        std::cout << "r-o: " << YamlHelper::parseValueToString( Option<_T>::readOnly ) << ", ";
        std::cout << "min: " << YamlHelper::parseValueToString( lowerBound ) << ", ";
        std::cout << "max: " << YamlHelper::parseValueToString( upperBound );
        std::cout << "): " << Option<_T>::getDescription() << std::endl;
    }
};


} // namespace


#endif /* BOUNDSOPTION_H_ */


