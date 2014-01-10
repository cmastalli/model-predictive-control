%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% poster.cls
%
% Document class to produce posters with KTH logo.
% 
% Mats Bengtsson, 28/5 2002
%
% Incompatible change: Added environment rutor to
% set the number of columns. Mats 29/7 2002
%
% Renamed to KTHEEposter and modified to adhere somewhat more to the
% graphical profile. Mats Bengtsson, 7/8 2006. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{KTHEEposter}[2006/08/07 -- EE Poster]

\RequirePackage[dvips]{color}
\usepackage{fancybox}
\RequirePackage{pstricks,pst-node}
\RequirePackage{graphicx}
\RequirePackage{multicol}
\RequirePackage{calc}

% Default size: A0
\newcommand{\p@persize}{a0}
% Handle A1 paper as well: 
\DeclareOption{a1}{\renewcommand{\p@persize}{a1}}
\DeclareOption{A1}{\renewcommand{\p@persize}{a1}}
\DeclareOption*{\PassOptionsToClass{\CurrentOption}{a0poster}}
\ProcessOptions
\LoadClass[\p@persize]{a0poster}

\newlength{\ruthjd}
\newlength{\rutbredd}
\newlength{\tmpl@ngth}

\newcmykcolor{KTHblue}{1 0.55 0 0}

%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Usage: \begin{rutor}{<number of columns>} ... \end{rutor}
%
\newenvironment{rutor}[1]{
  \begin{multicols*}{#1} \raggedright}
  {\end{multicols*}}


%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Usage \begin{ruta}[<number in each column>]{<height>}{<node name>} 
% ... \end{ruta}
% 
% The sum of the height values in each column should equal the
% number of frames in the column, i.e. height=1 always works.
% The optional first argument has default value=2 and should be the
% same for all frames in the same column. Different columns could
% contain different number of frames, though.
% The node name specifies a label that can be used when drawing arrows
% between the frames using pst-node.
%
\newenvironment{ruta}[3][2]{\begin{Sbox}%
    % Total height below the title,
    % paper height - margin - logo - space below logo - title:
    \setlength{\tmpl@ngth}{\paperheight-2in-\topmargin-8cm-2cm-20cm}
    \setlength{\ruthjd}{(\tmpl@ngth-#1\parskip)/#1-40.2mm}%
    \setlength{\rutbredd}{\columnwidth}
    \addtolength{\rutbredd}{-77mm}
    \global\def\name{#3}%
    \begin{minipage}[t][#2\ruthjd]{\rutbredd}}
    {\end{minipage}\end{Sbox}%
    \rnode{\name}{\psframebox[linewidth=5mm,linecolor=KTHblue,framearc=.1,framesep=15mm,fillstyle=solid,fillcolor=white,shadow=true,shadowcolor=gray,shadowsize=12mm,shadowangle=-30]{\TheSbox}}
}

\newenvironment{ruta_utan_ram}[2][2]{%
  % Total height below the title,
  % paper height - margin - logo - space below logo - title:
  \setlength{\tmpl@ngth}{\paperheight-2in-\topmargin-8cm-2cm-20cm}
  \setlength{\ruthjd}{(\tmpl@ngth-#1\parskip)/#1-40.2mm}%
  \setlength{\rutbredd}{\columnwidth}
  \addtolength{\rutbredd}{-57mm}
  \vskip 20mm\hspace{20mm}\begin{minipage}[t][#2\ruthjd]{\rutbredd}}
  {\end{minipage}}

\parskip 1cm
\columnsep 0pt

\renewcommand{\maketitle}{
  \centering
  \includegraphics[height=8cm]{kth_cmyk_electr_engine}\\[2cm]
  %
  % Move to the left edge of the paper
  \setlength{\tmpl@ngth}{-\hoffset-1in-\oddsidemargin}%
  \noindent\mbox{}\hspace*{\tmpl@ngth}%
  \colorbox[cmyk]{1,0.55,0,0}{\parbox[t][200mm]{\paperwidth}{
      \centering\mbox{}\vfill\color{white}
      \begin{minipage}{0.8\textwidth}
        \begin{center}
          \VERYHuge\bfseries
          \@title \\[3cm]
          \Huge \itshape
          \@author
        \end{center}
      \end{minipage}
      \vfill\mbox{}}}
}

