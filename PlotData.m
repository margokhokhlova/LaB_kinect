function PlotData( XYZ )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
ConnectionsUPCVk2

% Set to 1 for keeping skeleton in (0,0,0)
static    = 1;

%display using their function

        % for all frames
        for k= 1: (size(XYZ,1)/25)
            
            td=XYZ((k-1)*25+1:k*25,:);
             if(static==1)
                td = td - repmat(td(1,:),[25,1]);
                
            end
            
            
            clf;
          
            axis([-2 2 -3 3 -2 2]);
            view([-170, -50]); %  
            hold on
            scatter3(td(:,1),td(:,2),td(:,3),'.');
            grid on
            
            for kk=1:size(joints,1)
                plot3([td(joints(kk,1),1) ,td(joints(kk,2),1) ],...
                    [td(joints(kk,1),2) ,td(joints(kk,2),2) ],...
                    [td(joints(kk,1),3) ,td(joints(kk,2),3) ],'b');
                
                
            end
  
            
            pause(0.01);
        end






end

