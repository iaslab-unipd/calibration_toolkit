/*
 *  Copyright (c) 2013-2014, Filippo Basso <bassofil@dei.unipd.it>
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *     1. Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *     2. Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *     3. Neither the name of the copyright holder(s) nor the
 *        names of its contributors may be used to endorse or promote products
 *        derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef KINECT_DEPTH_TWO_STEPS_MODEL_H_
#define KINECT_DEPTH_TWO_STEPS_MODEL_H_

#include <calibration_common/objects/globals.h>
#include <calibration_common/base/matrix.h>
#include <calibration_common/base/math.h>

#include <calibration_common/depth/undistortion_model.h>

namespace calibration
{

template <typename ScalarT_, typename LocalT_, typename GlobalT_>
  class TwoStepsModel;

template <typename ScalarT_, typename LocalT_, typename GlobalT_>
  struct ModelTraits<TwoStepsModel<ScalarT_, LocalT_, GlobalT_> >
  {
    typedef ScalarT_ Scalar; // TODO derive
    struct Data
    {
      typedef typename ModelTraits<LocalT_>::Data LocalData;
      typedef typename ModelTraits<GlobalT_>::Data GlobalData;

      boost::shared_ptr<LocalData> local_data_;
      boost::shared_ptr<GlobalData> global_data_;
    };

  };

template <typename ScalarT_, typename LocalT_, typename GlobalT_>
  class TwoStepsModel : public DepthUndistortionModel<TwoStepsModel<ScalarT_, LocalT_, GlobalT_> >
  {
  public:

    typedef boost::shared_ptr<TwoStepsModel> Ptr;
    typedef boost::shared_ptr<const TwoStepsModel> ConstPtr;

    typedef ModelTraits<TwoStepsModel> Traits;
    typedef DepthUndistortionModel<TwoStepsModel> Base;

    typedef typename Traits::Scalar Scalar;
    typedef typename Traits::Data Data;

    TwoStepsModel() :
      data_(boost::make_shared<Data>())
    {
      // Do nothing
    }

    virtual ~TwoStepsModel()
    {
      // Do nothing
    }

    virtual void setData(const boost::shared_ptr<Data> & data)
    {
      assert(local_ and global_);
      data_ = data;
      local_->setData(data->local_data_);
      global_->setData(data->global_data_);
    }

    virtual const boost::shared_ptr<Data> & data() const
    {
      return data_;
    }

    void setLocalModel(const boost::shared_ptr<LocalT_> & local)
    {
      local_ = local;
      data_->local_data_ = local->data();
    }

    void setGlobalModel(const boost::shared_ptr<GlobalT_> & global)
    {
      global_ = global;
      data_->global_data_ = global->data();
    }

    const boost::shared_ptr<LocalT_> & localModel() const
    {
      return local_;
    }

    const boost::shared_ptr<GlobalT_> & globalModel() const
    {
      return global_;
    }

  protected:

    boost::shared_ptr<LocalT_> local_;
    boost::shared_ptr<GlobalT_> global_;

    boost::shared_ptr<Data> data_;

  };

} /* namespace calibration */
#endif /* KINECT_DEPTH_TWO_STEPS_MODEL_H_ */
