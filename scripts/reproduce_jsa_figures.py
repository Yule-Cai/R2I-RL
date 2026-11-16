#!/usr/bin/env python3
"""Regenerate plots in an explicitly new directory, preserving frozen assets."""
from pathlib import Path
import argparse,shutil,subprocess,sys,tempfile
ROOT=Path(__file__).resolve().parents[1];DATA=ROOT/'artifacts/jsa_20261116'
def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',required=True,type=Path);a=p.parse_args();out=a.out.resolve();assert not out.exists(),'Use a fresh output directory.'
 with tempfile.TemporaryDirectory(prefix='rulesync_figures_') as tmp:
  t=Path(tmp);shutil.copy2(DATA/'make_figures.py',t/'make_figures.py');(t/'source_data').symlink_to(DATA/'source_data',target_is_directory=True)
  subprocess.run([sys.executable,str(t/'make_figures.py')],cwd=t,check=True)
  out.mkdir(parents=True)
  for n in ['figures','figure_previews']:shutil.copytree(t/n,out/n)
  for n in ['study_numbers.tex','main_results_table.tex','authorization_results_table.tex']:shutil.copy2(t/n,out/n)
 print(f'Regenerated plots from pinned inputs: {out}')
if __name__=='__main__':main()
