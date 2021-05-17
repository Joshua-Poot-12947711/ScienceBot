classdef DobotEnvironment < handle
    properties        
        workspace = [-2 2 -2 2 -2 2]
    end
    methods (Static)
        %loads the environment objects like the table, estop, etc
        function loadEnvironmentObjects(self)
            [faceData, vertexData, data] = plyread('environment.ply','tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            environmentMesh_h = trisurf(faceData,vertexData(:,1),vertexData(:,2), vertexData(:,3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            camlight;
            axis([-3 3 -3 3 -0.05 3]);
            view(3);
            axis vis3d
            hold on;
        end
        %% Generating the light screen
        function [vertex,face,faceNormals] = generateLightScreen(lower,upper,axis_h)
if nargin<3
        axis_h=gca;
end
hold on

vertex(1,:)=lower;
vertex(2,:)=[upper(1),lower(2:3)];
vertex(3,:)=[upper(1:2),lower(3)];
vertex(4,:)=[upper(1),lower(2),upper(3)];
vertex(5,:)=[lower(1),upper(2:3)];
vertex(6,:)=[lower(1:2),upper(3)];
vertex(7,:)=[lower(1),upper(2),lower(3)];
vertex(8,:)=upper;

face=[1,2,3;1,3,7;
     1,6,5;1,7,5;
     1,6,4;1,4,2;
     6,4,8;6,5,8;
     2,4,8;2,3,8;
     3,7,5;3,8,5;
     6,5,8;6,4,8];

if 2 < nargout    
    faceNormals = zeros(size(face,1),3);
    for faceIndex = 1:size(face,1)
        v1 = vertex(face(faceIndex,1)',:);
        v2 = vertex(face(faceIndex,2)',:);
        v3 = vertex(face(faceIndex,3)',:);
        faceNormals(faceIndex,:) = unit(cross(v2-v1,v3-v1));
    end
end
    links=[1,2;
        2,3;
        3,7;
        7,1;
        1,6;
        5,6;
        5,7;
        4,8;
        5,8;
        6,4;
        4,2;
        8,3];
    for i=1:size(links,1)
        plot3(axis_h,[vertex(links(i,1),1),vertex(links(i,2),1)],...
            [vertex(links(i,1),2),vertex(links(i,2),2)],...
            [vertex(links(i,1),3),vertex(links(i,2),3)],'k')
    end
    tcolor = [1 0 0];
    patch('Faces',face,'Vertices',vertex,'FaceVertexCData',tcolor,'FaceColor','flat','lineStyle','none');
        end
    end
end
    