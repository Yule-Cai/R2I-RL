"""Regenerate journal figures, tables and macros from retained evidence only."""
from collections import Counter
from pathlib import Path
import csv, hashlib, json
import matplotlib as mpl
mpl.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.patches import Polygon
import numpy as np
from PIL import Image

HERE=Path(__file__).resolve().parent
DATA=HERE/'source_data'; OUT=HERE/'figures'; PRE=HERE/'figure_previews'
OUT.mkdir(exist_ok=True);PRE.mkdir(exist_ok=True)
mpl.rcParams.update({'font.family':'sans-serif','font.sans-serif':['Arial','DejaVu Sans'],
    'font.size':8,'axes.titlesize':8.5,'axes.labelsize':8,'xtick.labelsize':8,'ytick.labelsize':8,
    'legend.fontsize':8,'pdf.fonttype':42,'ps.fonttype':42,'svg.fonttype':'none',
    'axes.spines.top':False,'axes.spines.right':False,'axes.linewidth':.7})
C={'full':'#326C9B','neutral':'#9CA7AF','green':'#35816D','orange':'#C58038','red':'#B45B5B','pale':'#EDF0F2'}
ARMS=['full','geometry_only','rho0','native_async','native_wait','native_periodic']
NAMES=['Full','Geometry only','Zero margin','Native async','Native wait','Native periodic']
rows=list(csv.DictReader((DATA/'formal2970.csv').open()))
coverage=json.loads((DATA/'formal_coverage.json').read_text())
outcomes=json.loads((DATA/'formal_outcomes.json').read_text())
auth=json.loads((DATA/'authorization12_review.json').read_text())
vol=json.loads((DATA/'volumetric140_review.json').read_text())
occ=json.loads((DATA/'occupancy_comparison.json').read_text())
assert len(rows)==len({r['id'] for r in rows})==2970
assert len({r['scene'] for r in rows})==23 and len({r['case_id'] for r in rows})==45
assert sum(r['raw_task_result']=='PASS' for r in rows)==1997
assert sum(r['epoch2_assessed']=='True' for r in rows)==coverage['epoch2_assessed_total']==2943
assert all(int(r['epoch2_body_entries'])==0 for r in rows if r['epoch2_assessed']=='True')
for arm in ARMS:
    rr=[r for r in rows if r['arm']==arm]
    assert len(rr)==outcomes['by_arm'][arm]['completed']
    assert sum(r['raw_task_result']=='PASS' for r in rr)==outcomes['by_arm'][arm]['task_pass']
assert len(auth['rows'])==auth['trials']==12 and auth['raw_outcomes_unchanged']
assert sum(r['explicit_identity_rejection'] for r in auth['rows'])==12
assert sum(r['raw_task_result']=='PASS' for r in auth['rows'])==8
assert len(vol)==140 and sum(r['epoch2_assessed'] for r in vol)==130
assert not any(r['actual_committed_raw_penetration'] for r in vol)
assert sum(r['old_audit_retired_rule_false_positive'] for r in vol)==20


def save(fig,name):
    # Fixed figure dimensions preserve the intended final type size.
    fig.savefig(OUT/(name+'.pdf'),dpi=600)
    fig.savefig(OUT/(name+'.svg'),dpi=600)
    fig.savefig(OUT/(name+'.tiff'),dpi=600,pil_kwargs={'compression':'tiff_lzw'})
    fig.savefig(PRE/(name+'.png'),dpi=180)
    plt.close(fig)


def formal():
    fig=plt.figure(figsize=(7.05,4.7))
    gs=fig.add_gridspec(2,2,width_ratios=[1.1,1],hspace=.59,wspace=.61,
                          left=.16,right=.98,bottom=.10,top=.92)
    ax=fig.add_subplot(gs[0,0])
    common={ (r['case_id'],r['speed'],r['seed']) for r in rows if r['arm']=='geometry_only'}
    assert len(common)==405
    counts=[]
    for arm in ARMS:
        rr=[r for r in rows if r['arm']==arm and (r['case_id'],r['speed'],r['seed']) in common]
        assert len(rr)==405
        counts.append(sum(r['raw_task_result']=='PASS' for r in rr))
    for y,(arm,k) in enumerate(zip(ARMS,counts)):
        ax.barh(y,k/405*100,height=.6,color=C['full'] if arm=='full' else C['neutral'])
        ax.text(k/405*100+2,y,str(k)+'/405',va='center')
    ax.set_yticks(range(6),NAMES);ax.invert_yaxis();ax.set_xlim(0,119)
    ax.set_xticks([0,50,100]);ax.set_xlabel('Task arrival (%)')
    ax.set_title('a  Common scene/route/speed/seed',loc='left',pad=12)
    ax=fig.add_subplot(gs[1,0])
    for y,arm in enumerate(ARMS):
        d=coverage['by_arm'][arm];n=d['rows'];k=d['epoch2_assessed']
        ax.barh(y,100*k/n,color=C['green'],height=.6)
        ax.barh(y,100*(n-k)/n,left=100*k/n,color=C['pale'],hatch='///',edgecolor=C['neutral'],height=.6)
        ax.text(102,y,f'{k}/{n}',va='center')
    ax.set_yticks(range(6),NAMES);ax.invert_yaxis();ax.set_xlim(0,129)
    ax.set_xticks([0,50,100]);ax.set_xlabel('Post-update geometry assessed (%)')
    ax.set_title('c  Coverage of every planned attempt',loc='left',pad=12)
    ax=fig.add_subplot(gs[:,1]);scenes=sorted({r['scene'] for r in rows});speeds=['0.1','0.2','0.25']
    mat=np.zeros((len(scenes),3));cells=[]
    for i,scene in enumerate(scenes):
        cc=[]
        for j,speed in enumerate(speeds):
            rr=[r for r in rows if r['arm']=='full' and r['scene']==scene and r['speed']==speed]
            k=sum(r['raw_task_result']=='PASS' for r in rr);mat[i,j]=k/len(rr);cc.append((k,len(rr)))
        cells.append(cc)
    im=ax.imshow(mat,vmin=0,vmax=1,cmap='Blues',aspect='auto',interpolation='nearest')
    ax.set_yticks(range(len(scenes)),[s.removeprefix('hssd_').split('_')[0] for s in scenes])
    ax.set_xticks(range(3),['0.10','0.20','0.25']);ax.set_xlabel('Commanded speed (m/s)')
    for i in range(len(scenes)):
        for j in range(3):
            k,n=cells[i][j];ax.text(j,i,f'{k}/{n}',ha='center',va='center',color='white' if mat[i,j]>.65 else '#222',fontsize=8)
    ax.set_title('b  Full-arm arrival by layout',loc='left',pad=12)
    save(fig,'f4_formal_results')


def authorization():
    rr=sorted(auth['rows'],key=lambda r:(r['scene'],r['case_id'],r['probe_kind']))
    fields=['positive_epoch2_admission','explicit_identity_rejection','fresh_readmission_and_release','raw_task_result','raw_error']
    fig,ax=plt.subplots(figsize=(7.05,3.35));fig.subplots_adjust(left=.27,right=.98,bottom=.24,top=.80)
    for i,r in enumerate(rr):
        vals=[bool(r[fields[0]]),bool(r[fields[1]]),bool(r[fields[2]]),r[fields[3]]=='PASS',r[fields[4]]=="AssertionError('probe caused executable path release')"]
        for j,v in enumerate(vals):
            ax.scatter(j,i,s=43,marker='o' if v else 'x',facecolors=(C['orange'] if j==4 else C['green']) if v else C['neutral'],linewidths=.8)
        if i%2==0:ax.axhspan(i-.5,i+.5,color='#F4F6F7',zorder=-1)
    ax.set_yticks(range(len(rr)),[r['scene'][-6:]+'/'+r['case_id'].split('_')[-1]+'  '+('H' if r['probe_kind']=='late_authorization' else 'S') for r in rr])
    ax.set_xticks(range(5),['Current\nadmission','Identity\nrefusal','Fresh\nrelease','Task\narrival','Observer\nflag'])
    ax.xaxis.tick_top();ax.tick_params(axis='both',length=0);ax.invert_yaxis();ax.set_ylim(11.6,-.6);ax.set_xlim(-.5,4.5)
    for side in ax.spines.values():side.set_visible(False)
    for j,total in enumerate([12,12,9,8,3]):ax.text(j,12.4,str(total)+'/12',ha='center',va='top',fontweight='bold')
    fig.text(.02,.96,'Actual typed authorization probes',fontweight='bold',va='top',fontsize=9)
    fig.text(.02,.07,'S: same-path stale identity   H: historical envelope replay\nFilled circle: recorded endpoint/flag; cross: endpoint absent. Original outcomes: 8 PASS, 4 FAIL.',fontsize=8)
    save(fig,'f5_authorization')


def volumetric():
    fig,axs=plt.subplots(2,1,figsize=(3.45,4.15),gridspec_kw={'height_ratios':[1,1.12]})
    fig.subplots_adjust(left=.22,right=.96,top=.91,bottom=.20,hspace=.88)
    scenes=sorted(occ);vals=[occ[s]['0.80-1.50']['free2d_but_voxel_blocked_pct_of_free'] for s in scenes]
    ax=axs[0];ax.bar(range(3),vals,width=.55,color=C['orange'])
    for i,v in enumerate(vals):ax.text(i,v+.4,f'{v:.2f}%',ha='center')
    ax.set_ylim(0,15);ax.set_yticks([0,5,10,15]);ax.set_xticks(range(3),[s[-6:] for s in scenes])
    ax.set_ylabel('Projection-free cells\nblocked at flight height (%)')
    ax.set_title('a  Occupancy disagreement (0.80–1.50 m)',loc='left',pad=15)
    ax=axs[1]
    for i,mode in enumerate(['voxel','projected2d']):
        rr=[r for r in vol if r['row']['occ']==mode]
        parts=[sum(r['original_result']=='PASS' for r in rr),sum(r['old_audit_retired_rule_false_positive'] for r in rr),sum(not r['epoch2_assessed'] for r in rr)]
        assert sum(parts)==70
        bottom=0
        for k,col,hatch in zip(parts,[C['green'],C['orange'],C['pale']],['','///','xxx']):
            ax.bar(i,k,bottom=bottom,width=.55,color=col,hatch=hatch,edgecolor='#6E767C',linewidth=.4)
            if k:ax.text(i,bottom+k/2,str(k),ha='center',va='center')
            bottom+=k
        assessed=sum(r['epoch2_assessed'] for r in rr)
        ax.text(i,73,f'{assessed}/70 assessed',ha='center')
    ax.set_xticks([0,1],['Per-height\nvoxel','Lifted\nprojection']);ax.set_ylim(0,88);ax.set_yticks([0,35,70]);ax.set_ylabel('Original attempts')
    ax.set_title('b  Outcomes and snapshot assessment',loc='left',pad=13)
    fig.text(.05,.015,'Green: original PASS; hatched: post-hoc audit FAIL;\ncrosshatched: initial refusal. Active-snapshot entries: 0/130.',fontsize=8)
    save(fig,'f6_volumetric')


def tables():
    macros={'AuthTrials':auth['trials'],'AuthExercised':auth['exercised'],'AuthRejected':auth['explicit_identity_rejections'],'AuthTaskPass':auth['task_pass'],'AuthTaskFail':auth['task_fail'],'AuthReadmitted':auth['fresh_readmission_and_release']}
    (HERE/'study_numbers.tex').write_text('% Generated from retained source data.\n'+''.join('\\newcommand{\\'+k+'}{'+str(v)+'}\n' for k,v in macros.items()))
    lines=[r'\begin{table}[t]',r'\caption{All-attempt task outcomes and post-update geometry coverage.}',r'\label{tab:main}\centering\footnotesize',r'\begin{tabular}{@{}lrrr@{}}\toprule',r'Arm & Arrival & Assessed & Unassessed\\\midrule']
    for arm,name in zip(ARMS,NAMES):
        d=outcomes['by_arm'][arm];c=coverage['by_arm'][arm]
        lines.append(f"{name} & {d['task_pass']}/{d['completed']} & {c['epoch2_assessed']} & {c['epoch2_unassessed']}"+r'\\')
    lines += [r'\bottomrule\end{tabular}',r'\end{table}']
    (HERE/'main_results_table.tex').write_text('\n'.join(lines)+'\n')
    lines=[r'\begin{table}[t]',r'\caption{Authorization receiver endpoints, with all DEV attempts retained.}',r'\label{tab:auth}\centering\footnotesize',r'\begin{tabular}{@{}lrr@{}}\toprule',r'Endpoint & Same-path & Replay\\\midrule']
    sets=[[r for r in auth['rows'] if r['probe_kind']==kind] for kind in ['stale_identity','late_authorization']]
    for label,predicate in [('Current admission',lambda r:r['positive_epoch2_admission']),('Explicit identity refusal',lambda r:r['explicit_identity_rejection']),('Fresh readmission/release',lambda r:r['fresh_readmission_and_release']),('Task arrival',lambda r:r['raw_task_result']=='PASS'),('Original task FAIL',lambda r:r['raw_task_result']!='PASS'),('Redundant-observer flag',lambda r:r['raw_error']=="AssertionError('probe caused executable path release')")]:
        counts=[sum(predicate(r) for r in rr) for rr in sets]
        lines.append(label+f' & {counts[0]}/6 & {counts[1]}/6'+r'\\')
    lines += [r'\bottomrule\end{tabular}',r'\end{table}']
    (HERE/'authorization_results_table.tex').write_text('\n'.join(lines)+'\n')


def environments():
    base=DATA/'scene_recaptures';scenes=sorted(occ)
    if not all((base/s/'overview.png').exists() for s in scenes):return
    original=base
    fig,axs=plt.subplots(2,3,figsize=(7.05,4.3));fig.subplots_adjust(left=.035,right=.99,bottom=.08,top=.94,wspace=.12,hspace=.11)
    for j,scene in enumerate(scenes):
        receipt=json.loads((base/scene/'receipt.json').read_text());assert receipt['all_noncamera_content_equal']
        im=base/scene/'overview.png';assert hashlib.sha256(im.read_bytes()).hexdigest()==receipt['images']['overview']['sha256']
        axs[0,j].imshow(Image.open(im));axs[0,j].axis('off');axs[0,j].set_title(scene.removeprefix('hssd_'))
        mp=json.loads((original/scene/'map.json').read_text());case=json.loads((original/scene/'case.json').read_text())
        assert hashlib.sha256((original/scene/'map.json').read_bytes()).hexdigest()==receipt['map_sha256']==case['map_sha256']
        a=np.array(mp['data']).reshape(mp['height'],mp['width']);res=mp['resolution'];x,y=mp['origin'][:2]
        ax=axs[1,j];ax.imshow(a,origin='lower',extent=[x,x+res*mp['width'],y,y+res*mp['height']],cmap=ListedColormap(['#EEEEEE','#FFFFFF','#45525C']),vmin=-1,vmax=100,interpolation='nearest')
        # Keep unknown (-1), free (0) and occupied (100) visually distinct.
        values=np.where(a<0,0,np.where(a==0,1,2))
        ax.images[-1].set_data(values);ax.images[-1].set_clim(0,2)
        for key,col,style in [('initial_rule',C['full'],'--'),('replacement_rule',C['red'],'-')]:
            ax.add_patch(Polygon(case[key],facecolor=col,alpha=.7,edgecolor='#222',linestyle=style,linewidth=.8))
        ax.scatter(*case['spawn'],marker='o',color=C['green'],s=22,edgecolors='white',linewidths=.5,zorder=5)
        ax.scatter(*case['goal'],marker='*',color=C['orange'],s=40,edgecolors='#222',linewidths=.3,zorder=5)
        left=x+.05*res*mp['width'];bottom=y+.06*res*mp['height'];ax.plot([left,left+2],[bottom,bottom],c='#111',lw=2)
        ax.text(left+1,bottom+.25,'2 m',ha='center',fontsize=8,bbox={'facecolor':'white','alpha':.8,'edgecolor':'none','pad':.3})
        ax.set_aspect('equal');ax.set_xticks([]);ax.set_yticks([])
    fig.text(.05,.015,'Maps: green circle, start; orange star, goal; blue dashed region, initial rule; red region, replacement rule.',fontsize=8)
    save(fig,'f3_environments')


if __name__=='__main__':
    tables();formal();authorization();volumetric();environments()
    inventory={str(p.relative_to(HERE)):{'bytes':p.stat().st_size,'sha256':hashlib.sha256(p.read_bytes()).hexdigest()} for p in sorted(DATA.rglob('*')) if p.is_file()}
    (HERE/'source_data_manifest.json').write_text(json.dumps(inventory,indent=2)+'\n')
    print(json.dumps({'formal_rows':len(rows),'auth_trials':auth['trials'],'volumetric_rows':len(vol),'generated_figures':[p.name for p in OUT.glob('f*.pdf')]},indent=2))
