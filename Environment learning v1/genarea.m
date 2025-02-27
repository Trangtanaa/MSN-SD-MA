function Area1 = genarea()
I_raw=imread('image.png');
I_resized=imresize(I_raw,[101,101]);
I_gray=rgb2gray(I_resized);
%level = graythresh(I_gray);
%I_gray2=imbinarize(I_gray,level);
%imshowpair(I_raw,I_gray,'montage');
Area1=zeros(101,101);
for i =1:101
    for j=1:101
        if I_gray(i,j)>1
            Area1(i,j)=255;
        else
            Area1(i,j)=1;
        end
    end
end
%imshow(I_gray);
%image(Area1);