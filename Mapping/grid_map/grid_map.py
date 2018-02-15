"""

2D grid map sample

author: Atsushi Sakai (@Atsushi_twi)

"""

from scipy.stats import norm
import math


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

import numpy as np
import matplotlib.pyplot as plt

AREA_WIDTH = 30.0


def generate_gaussian_grid_map(ox, oy, reso):

    minx = min(ox) - AREA_WIDTH / 2.0
    miny = min(oy) - AREA_WIDTH / 2.0
    maxx = max(ox) + AREA_WIDTH / 2.0
    maxy = max(oy) + AREA_WIDTH / 2.0
    xw = round((maxx - minx) / reso)
    yw = round((maxy - miny) / reso)

    # calc each potential
    pmap = [[0.0 for i in range(yw)] for i in range(xw)]

    STD = 10.0  # standard diviation

    for ix in range(xw):
        for iy in range(yw):

            x = ix / reso + minx
            y = iy / reso + miny

            # Search minimum distance
            mindis = float("inf")
            for (iox, ioy) in zip(ox, oy):
                d = math.sqrt((iox - x)**2 + (ioy - y)**2)
                if mindis >= d:
                    mindis = d

            pdf = (1.0 - norm.cdf(mindis, 0.0, STD))
            pmap[ix][iy] = pdf

    draw_heatmap(pmap)
    plt.show()


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

        #  print(norm.cdf(x, mean, std))

    draw_heatmap(pmap)
    plt.show()


def draw_heatmap(data):
    data = np.array(data).T
    plt.pcolor(data, vmax=1.0, cmap=plt.cm.Blues)
    plt.axis("equal")


def main():
    print(__file__ + " start!!")

    ox = [0.0, 5.0, 0.0]
    oy = [0.0, 5.0, 10.0]
    reso = 1.0

    generate_gaussian_grid_map(ox, oy, reso)
    #  generate_ray_casting_grid_map(ox, oy, reso)


if __name__ == '__main__':
    main()
