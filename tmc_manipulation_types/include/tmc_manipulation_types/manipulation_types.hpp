/*
Copyright (c) 2012 TOYOTA MOTOR CORPORATION
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the disclaimer
below) provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its contributors may be used
  to endorse or promote products derived from this software without specific
  prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/
#ifndef TMC_MANIPULATION_TYPES_MANIPULATION_TYPES_HPP_
#define TMC_MANIPULATION_TYPES_MANIPULATION_TYPES_HPP_

#include <boost/serialization/nvp.hpp>
#include <boost/serialization/split_member.hpp>

#include <Eigen/Core>

namespace tmc_manipulation_types {

typedef std::vector<std::string> NameSeq;

struct JointState {
  NameSeq name;
  Eigen::VectorXd position;
  Eigen::VectorXd velocity;
  Eigen::VectorXd effort;

  bool Validate() const { return !name.empty() && position.size() == name.size(); }

 private:
  friend class boost::serialization::access;
  template <class Archive> void save(Archive& archive,
                                     const unsigned int version) const {
    uint32_t size = name.size();
    archive & boost::serialization::make_nvp("Size", size);
    for (uint32_t i = 0; i < name.size(); i++) {
      std::string name_tag("Name" + boost::lexical_cast<std::string>(i));
      std::string value_tag("Value" + boost::lexical_cast<std::string>(i));
      archive & boost::serialization::make_nvp(name_tag.c_str(), name.at(i));
      archive & boost::serialization::make_nvp(value_tag.c_str(), position(i));
    }
  }
  template <class Archive> void load(Archive& archive,
                                     const unsigned int version) {
    uint32_t size;
    archive & boost::serialization::make_nvp("Size", size);
    name.resize(size);
    position.resize(size);
    for (uint32_t i = 0; i < name.size(); i++) {
      std::string name_tag("Name" + boost::lexical_cast<std::string>(i));
      std::string value_tag("Value" + boost::lexical_cast<std::string>(i));
      archive & boost::serialization::make_nvp(name_tag.c_str(), name.at(i));
      archive & boost::serialization::make_nvp(value_tag.c_str(), position(i));
    }
  }
  BOOST_SERIALIZATION_SPLIT_MEMBER();
};


enum BaseMovementType {
  kFloat,
  kPlanar,
  kRailX,
  kRailY,
  kRailZ,
  kRotationX,
  kRotationY,
  kRotationZ,
  kNone,
};

}  // namespace tmc_manipulation_types

#endif  // TMC_MANIPULATION_TYPES_MANIPULATION_TYPES_HPP_
