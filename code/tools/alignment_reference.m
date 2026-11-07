function alignment_reference(input_file,output_file,source_root,harness_dir)
% Run original numerical MATLAB sources; harness_dir only adapts fixed control dimensions.
addpath(fullfile(source_root,'bluerov2_kprm'));
addpath(fullfile(source_root,'bluerov2_lazybts'));
addpath(harness_dir,'-begin');
load(input_file);
for j=1:numel(models)
 m=models{j};f=forces{j};
 dx=zeros(size(states));next=zeros(size(states));
 for k=1:size(states,2)
  dx(:,k)=bluerov2Dynamics(states(:,k),f(:,k),m);
  next(:,k)=rk4StepLBTS(states(:,k),f(:,k),dt(k),m);
 end
 dynamics{j}=dx;rk4{j}=next;
 linearizer=struct('mode','finite-difference','model',m,'finiteDifferenceStep',1e-6);
 [~,A{j},B{j}]=evaluateRK4Linearization(linearizer,states(:,1),f(:,1),dt(1));
 edge={X,3,U{j},struct('time',times)};
 prop{j}=propagateBelief(edge,P0,Pt0,m,world,options{j},linearizer,Z);
end
[risk,risk_steps]=chanceCollisionProbability(chance_x,{chance_P},chance_world,Z,true,struct('visualize',false));
h=backwardDijkstra(graph,5);
% Extract this local helper verbatim from executeLBTSPlanHardware.m for six-input algebra tests.
feedback=zeros(6,size(feedback_state,2));errors=zeros(12,size(feedback_state,2));
for k=1:size(feedback_state,2)
 errors(:,k)=source_stateDifference(feedback_state(:,k),feedback_ref(:,k));
 feedback(:,k)=min(thrust_max,max(thrust_min,feedback_u(:,k)-feedback_K(:,:,k)*errors(:,k)));
end
engine_version=version;
save('-v7',output_file,'dynamics','rk4','A','B','prop','risk','risk_steps','h','feedback','errors','engine_version');
end
