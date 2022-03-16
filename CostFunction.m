function Fitness = CostFunction(I,particle,ImageIndex)
    
    particleImage = imcrop(I,[particle(1) particle(2) particle(3) particle(4)]);

%     figure
%     imshow(particleImage)
%     doubI = double(I);
%     tempI = zeros(size(I,1),size(I,2));
%     tempI(particle(1):particle(1)+particle(3),particle(2):particle(2)+particle(4)) = ones(particle(3)+1,particle(4)+1);
%     tempParticle = tempI.*doubI;

    maximum = max(max(particleImage));
    [targetCy,targetCx]=find(particleImage==maximum);

    try
        targetCx = particle(1) + targetCx(1);
        targetCy = particle(2) + targetCy(1);
    catch
        disp('you did something wrong')
    end



    try
        targetImage = imcrop(I,[targetCx-particle(5)/2 targetCy-particle(6)/2 particle(5) particle(6)]);
    catch
        disp('you did something wrong')
    end


    if(ImageIndex == 1)
        figure
        imshow(I)
        drawrectangle('Position',[targetCx-particle(5)/2,targetCy-particle(6)/2,particle(5),particle(6)],'StripeColor','r');
        drawrectangle('Position',[particle(1),particle(2),particle(3),particle(4)],'StripeColor','b');
        title("Found Target")
    end
    
    targetImage = double(targetImage);
    
    stdx = (size(targetImage,1)/2)^2;
    stdx = stdx/4.6;
    
    stdy = (size(targetImage,2)/2)^2;
    stdy = stdy/4.6;
    
    
    for i = 1:size(targetImage,1)
        for j = 1:size(targetImage,2)
            temp = ((i-size(targetImage,1)/2)^2)/stdx+((j-size(targetImage,2)/2)^2)/stdy;
%             temp = ((i-size(targetImage,1)/2)^2)+((j-size(targetImage,2)/2)^2);
            T(i,j) = exp(-temp);
            N(i,j) = 1-T(i,j);
        end
    end
%     T = 5*T;
%     N = 5*N;

    try
        F = abs(1/sum(T,"all")*sum(T.*targetImage,"all")-1.4/sum(N,"all")*sum(N.*targetImage,"all"))^2;
    catch
        disp('you did something wrong')
    end

    Fitness = F*(1+0.2/(size(targetImage,1)*size(targetImage,2)));

end


