"""

2D grid map sample

author: Atsushi Sakai (@Atsushi_twi)

"""


#  function gm=RayCastingUpdate(gm,z)
#  %レイキャスティングによるGridの更新

#  %事前レイキャスティングモデルの作成
#  gm=PreCasting(gm,z.ANGLE_TICK);

#  rayId=0;
#  %事前レイキャスティングモデルに従ってグリッドの確率の更新
#  for iz=1:length(z.data(:,1))%それぞれの観測点に対して
#  range=z.data(iz,1);

#  rayId=rayId+1;%レイキャスティングクラスタにおけるデータID
#  %各観測点はそれぞれのクラスタから取得できるとする。

#  %クラスタ内の各gridのデータからビームモデルによるupdate
#  for ir=1:length(gm.precasting(rayId).grid(:,1))
#  grange=gm.precasting(rayId).grid(ir,1);
#  gid=gm.precasting(rayId).grid(ir,5);

#  if grange<(range-gm.RESO/2) %free
#  gm.data(gid)=0;
#  elseif grange<(range+gm.RESO/2) %hit
#  gm.data(gid)=1;
#  end %それ以上の距離のgridはunknownなので何もしない
#  end
#  end

#  function gm=PreCasting(gm,angleTick)
#  %事前レイキャスティングモデルの作成

#  %各角度について対応するグリッドを追加していく
#  precasting=[];%プレキャスティングの結果 [最小角度,最大角度,中に入るgridのデータ]
#  for ia=(0-angleTick/2):angleTick:(360+angleTick/2)
#  %角度範囲の保存
#  ray.minAngle=ia;
#  ray.maxAngle=ia+angleTick;
#  grid=[];%角度範囲に入ったグリッドのデータ
#  for ig=1:(gm.nGrid)
#  %各グリッドのxy値を取得
#  gxy=GetXYFromDataIndex(ig,gm);
#  range=norm(gxy);
#  angle=atan2(gxy(2),gxy(1));
#  if angle<0 %[0 360]度に変換
#  angle=angle+2*pi;
#  end
#  if ray.minAngle<=toDegree(angle) && ray.maxAngle>=toDegree(angle)
#  grid=[grid;[range,angle,gxy,ig]];
#  end
#  end
#  %rangeの値でソーティングしておく
#  if ~isempty(grid)
#  ray.grid=sortrows(grid,1);
#  end
#  precasting=[precasting;ray];
#  end
#  gm.precasting=precasting;%Grid Mapデータに追加


#  function gm=LikelihoodUpdate(gm,z)
#  %尤度場のGridMapを作る関数

#  for ig=1:(gm.nGrid-1)
#  gxy=GetXYFromDataIndex(ig,gm);%それぞれのグリッドxyインデックスを取得
#  zxy=FindNearest(gxy,z);%最近傍の観測値の取得
#  p=GaussLikelihood(gxy,zxy);%ガウシアン尤度の計算
#  gm.data(ig)=p*10;%グリッドへの格納
#  end

#  function p=GaussLikelihood(gxy,zxy)
#  %ガウス分布の尤度を計算する関数
#  Sigma=diag([3,3]);%共分散行列
#  p=det(2*pi*Sigma)^(-0.5)*exp(-0.5*(gxy-zxy)*inv(Sigma)*(gxy-zxy)');

#  function zxy=FindNearest(xy,z)
#  %ある座標値xyに一番近いzの値を返す関数

#  %すべてのzとxyの差を計算
#  d=z.data(:,3:4)-repmat(xy,length(z.data(:,1)),1);

#  %ノルム距離の最小値のインデックスを取得
#  min=inf;%最小値
#  minid=0;
#  for id=1:length(d(:,1))
#  nd=norm(d(id,:));
#  if min>nd
#  min=nd;
#  minid=id;
#  end
#  end
#  zxy=z.data(minid,3:4);

#  function xy=GetXYFromDataIndex(ig,gm)
#  %Gridのデータインデックスから,そのグリッドのx,y座標を取得する関数

#  %x,yインデックスの取得
#  indy=rem(ig,gm.WIDTH)-1;
#  indx=fix(ig/gm.WIDTH);

#  x=GetXYPosition(indx,gm.WIDTH,gm.RESO);
#  y=GetXYPosition(indy,gm.HEIGHT,gm.RESO);
#  xy=[x y];

#  function position=GetXYPosition(index, width, resolution)
#  %x-yインデックスの値から、位置を取得する関数
#  position=resolution*(index-width/2)+resolution/2;

#  function gm=HitGridUpdate(gm,z)
#  %観測点がヒットしたグリッドの確率を1にする関数

#  for iz=1:length(z.data(:,1))
#  zx=z.data(iz,3);
#  zy=z.data(iz,4);
#  ind=GetDBIndexFromXY(zx,zy,gm);
#  gm.data(ind)=1.0;
#  end
#  gm.data=Normalize(gm.data);%正規化

#  function z=GetObservation()
#  %観測点をセンサのモデルに基いて、ランダムに取得する関数
#  z.data=[];% 観測値[range, angle x y;...]
#  z.ANGLE_TICK=10;%スキャンレーザの角度解像度[deg]
#  z.MAX_RANGE=50;%スキャンレーザの最大観測距離[m]
#  z.MIN_RANGE=5;%スキャンレーザの最小さい観測距離[m]

#  for angle=0:z.ANGLE_TICK:360
#  range=rand()*(z.MAX_RANGE-z.MIN_RANGE)+z.MIN_RANGE;
#  rad=toRadian(angle);
#  x=range*cos(rad);
#  y=range*sin(rad);
#  z.data=[z.data;[range rad x y]];
#  end

import numpy as np
import matplotlib.pyplot as plt

AREA_WIDTH = 30.0


def generate_ray_casting_grid_map(ox, oy, reso):

    minx = min(ox) - AREA_WIDTH / 2.0
    miny = min(oy) - AREA_WIDTH / 2.0
    maxx = max(ox) + AREA_WIDTH / 2.0
    maxy = max(oy) + AREA_WIDTH / 2.0
    xw = round((maxx - minx) / reso)
    yw = round((maxy - miny) / reso)

    # calc each potential
    pmap = [[0.0 for i in range(yw)] for i in range(xw)]

    for (x, y) in zip(ox, oy):

        ix = round(x * reso - minx)
        iy = round(y * reso - miny)

        pmap[ix][iy] = 100.0

    draw_heatmap(pmap)
    plt.show()


def draw_heatmap(data):
    data = np.array(data).T
    plt.pcolor(data, vmax=100.0, cmap=plt.cm.Blues)
    plt.axis("equal")


def main():
    print(__file__ + " start!!")

    ox = [0.0, 5.0]
    oy = [0.0, 5.0]
    reso = 1.0

    generate_ray_casting_grid_map(ox, oy, reso)


if __name__ == '__main__':
    main()
