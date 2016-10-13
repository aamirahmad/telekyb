

#include <StateEstimators/MeasureHandler.hpp>
#include <EigenTools.hpp>


namespace telekyb_state {

MeasureHandlerOption::MeasureHandlerOption()
	: OptionContainer("MeasureHandler")
{
	//viconTopic = addOption<std::string>("viconTopic", "Vicon data topic name", "TKVicon", false, true);

	Eigen::Matrix<double,6,6> qt;
	qt.Zero();
	vicCov = addOption<Eigen::Matrix<double,6,6> >("vicCov", "Vicon noise covariance matrix", qt, false, true);

	vicPosition = addOption<Eigen::Vector3d>("initState", "Initial estimate for Vicon position", Eigen::Vector3d(0.0,0.0,0.0), false, true);
	vicOrientation = addOption<Eigen::Quaterniond>("vicOrientation", "Initial estimate for Vicon orientation", Eigen::Quaterniond(1.0,0.0,0.0,0.0), false, true);
}

// Correction function ====================================================
void MeasureHandler::update(StateBufferElement& state, const MeasureBufferElement& z)
{
	// Eigen tools;
	EigenTools tools;

	//read the state
	Eigen::Vector3d  r(state.state.pose.position.x,state.state.pose.position.y,state.state.pose.position.z);
	Eigen::Vector3d dr(state.state.twist.linear.x,state.state.twist.linear.y,state.state.twist.linear.z);
	Eigen::Quaterniond q(state.state.pose.orientation.w,state.state.pose.orientation.x,state.state.pose.orientation.y,state.state.pose.orientation.z);

	//read the measure
	Eigen::Vector3d  zr(state.state.pose.position.x,state.state.pose.position.y,state.state.pose.position.z);
	Eigen::Quaterniond zq(state.state.pose.orientation.w,state.state.pose.orientation.x,state.state.pose.orientation.y,state.state.pose.orientation.z);
	Eigen::Matrix<double, 6, 6> Qt = options.vicCov->getValue();

	Eigen::Vector3d BrBM = options.vicPosition->getValue();
	Eigen::Quaterniond BqM = options.vicOrientation->getValue();
	Eigen::Quaterniond WqM = q*BqM;

    Eigen::Matrix<double, 6, 9> H;
    H << Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Zero(), tools.gamma(q,BrBM)*tools.jacqp(q),
    		 Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(),        Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 6, 6> Hn;
    Hn.Identity();

    Eigen::Matrix<double, 9, 6> K = state.covariance*H.transpose()*(H*state.covariance*H.transpose()+Hn*Qt*Hn.transpose());


    Eigen::Vector3d perr = tools.toRodrigues(zq*WqM.conjugate());

    Eigen::Matrix<double, 6, 1> error;
    error << zr-(r+q*BrBM),
                      perr;
    Eigen::Matrix<double, 9, 1> update = K*error;

    r+=update.block<3,1>(0,0);
    dr+=update.block<3,1>(3,3);
    q = tools.toQuaternion((Eigen::Vector3d)update.block<3,1>(6,0))*q;

    state.state.pose.position.x = r(0);
    state.state.pose.position.y = r(1);
    state.state.pose.position.z = r(2);

    state.state.twist.linear.x = dr(0);
    state.state.twist.linear.y = dr(1);
    state.state.twist.linear.z = dr(2);

    state.state.pose.orientation.w = q.w();
	state.state.pose.orientation.x = q.x();
	state.state.pose.orientation.y = q.y();
	state.state.pose.orientation.z = q.z();

    //sigma = sigma_prd-K*H*sigma_prd;
    state.covariance = (Eigen::Matrix<double, 9, 9>::Identity()-K*H)*state.covariance*(Eigen::Matrix<double, 9, 9>::Identity()-K*H).transpose() + K*Qt*K.transpose();
}


} /* namespace telekyb_state */
