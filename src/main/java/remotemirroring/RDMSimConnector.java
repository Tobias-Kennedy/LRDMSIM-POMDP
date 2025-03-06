package remotemirroring;


import program.POMDP;
// import rdm.management.Effector;
// import rdm.management.NetworkManagment;
// import rdm.management.Probe;
// import rdm.network.Monitorables;

import java.util.List;

import org.lrdm.TimedRDMSim;
import org.lrdm.probes.*;
import org.lrdm.topologies.TopologyStrategy;
import org.lrdm.effectors.*;

import solver.BeliefPoint;

public class RDMSimConnector {
	
	// public static NetworkManagment network_management;
	public static TimedRDMSim timedRDMSim;
	public static boolean refsetcreation=false;
	public static LinkProbe lp;
	public static MirrorProbe mp;
	public static Effector effector;
	public static int timestep;
	
	public static POMDP p;
	
	
	
	
	public RDMSimConnector()
	{
		// network_management = new NetworkManagment();
		// network_management = new Network(null, timestep, timestep, timestep, null)
		lp = timedRDMSim.getLinkProbe();
		mp = timedRDMSim.getMirrorProbe();

		effector = timedRDMSim.getEffector();
	}
	
	public int performAction(int selectedaction)
	{
		
	////Perform ITP or DTP on the link on the simulator
			///return rewards and observations
			//update belief value and change initial belief
			
			///Immediate Reward
			double r=p.getReward(p.getCurrentState(), selectedaction);
			int nextstate;
			//update state for pomdp in nextStateRDM	
			nextstate=p.nextStateRDM(p.getCurrentState(), selectedaction);
			p.setCurrentState(nextstate);
			
			///Observation
			int obs=p.getObservation(selectedaction, nextstate);
			BeliefPoint b=p.updateBelief(p.getInitialBelief(), selectedaction, obs);
			p.setInitialBelief(b);
			
			//p.getReward(s, action);
			
			/*S currentS  = states.stateIdentifier(currentState);
			
			S nextState = this.transitions.nextState(currentS, action);
			
			this.currentState = states.stateNumber(nextState);
			
			
			double[] reward = this.rewards.getReward(currentS, action, nextState);
			
			
			
				O obs = this.observationFunction.getObservation(action, nextState);
				
				this.beliefUpdate(action, obs);*/
		
			
			
			return 0;

			
		
	}
	
	

}
