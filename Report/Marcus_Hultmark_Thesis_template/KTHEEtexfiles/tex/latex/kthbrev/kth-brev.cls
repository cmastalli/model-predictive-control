%%
%% This is file `kth-brev.cls',
%% generated with the docstrip utility.
%%
%% The original source files were:
%%
%% kthbrev.dtx  (with options: `classfile,swedish')
%% 
%% IMPORTANT NOTICE:
%% 
%% For the copyright see the source file.
%% 
%% Any modified versions of this file must be renamed
%% with new filenames distinct from kth-brev.cls.
%% 
%% For distribution of the original source see the terms
%% for copying and modification in the file kthbrev.dtx.
%% 
%% This generated file may be distributed as long as the
%% original source files, as listed above, are part of the
%% same distribution. (The sources need not necessarily be
%% in the same archive or directory.)
\NeedsTeXFormat{LaTeX2e}[2000/01/01]
\ProvidesClass{kth-brev}
              [2006/03/25 v1.5 KTH letters]
\DeclareOption{a4paper}{}
\DeclareOption{a5paper}{\OptionNotUsed}
\DeclareOption{b5paper}{\OptionNotUsed}
\DeclareOption{letterpaper}{\OptionNotUsed}
\DeclareOption{legalpaper}{\OptionNotUsed}
\DeclareOption{executivepaper}{\OptionNotUsed}
\DeclareOption{landscape}{\OptionNotUsed}
\setlength\paperheight{297mm}
\setlength\paperwidth {210mm}
\DeclareOption{10pt}{\renewcommand\@ptsize{0}}
\DeclareOption{11pt}{\renewcommand\@ptsize{1}}
\DeclareOption{12pt}{\renewcommand\@ptsize{2}}
\newcommand{\@ptsize}{1}
\DeclareOption{twoside}{\@twosidetrue  \@mparswitchtrue}
\DeclareOption{oneside}{\@twosidefalse \@mparswitchfalse}
\DeclareOption{draft}{\setlength\overfullrule{5pt}}
\DeclareOption{final}{\setlength\overfullrule{0pt}}
\DeclareOption{leqno}{\input{leqno.clo}}
\DeclareOption{fleqn}{\input{fleqn.clo}}
\newif\if@nadabrev@head \@nadabrev@headtrue
\newif\if@nadabrev@foot \@nadabrev@foottrue
\DeclareOption{nohead}{\@nadabrev@headfalse}
\DeclareOption{nofoot}{\@nadabrev@footfalse}
\newcommand{\@nadabrev@layout}{KTH}
\DeclareOption{DIN}{\renewcommand\@nadabrev@layout{DIN}}
\DeclareOption{SS}{\renewcommand\@nadabrev@layout{SS}}
\newcommand{\@nadabrev@logo}{\@nadabrev@default@logo}
\newcommand{\@nadabrev@logocol}{cmyk}
\newcommand{\@nadabrev@logoname}{}
\newif\if@nadabrev@forcedcolor \@nadabrev@forcedcolorfalse
\newcommand{\logotype}[1]{\renewcommand{\@nadabrev@logo}{#1}}
\newcommand{\logoname}[1]{\renewcommand{\@nadabrev@logoname}{#1}}
\DeclareOption{color}{%
  \@nadabrev@forcedcolortrue}
\DeclareOption{colour}{%
  \@nadabrev@forcedcolortrue}
\DeclareOption{bw}{%
  \@nadabrev@forcedcolortrue
  \renewcommand{\@nadabrev@logocol}{svv}}
\DeclareOption{b&w}{%
  \@nadabrev@forcedcolortrue
  \renewcommand{\@nadabrev@logocol}{svv}}
\DeclareOption{svv}{%
  \@nadabrev@forcedcolortrue
  \renewcommand{\@nadabrev@logocol}{svv}}
\ProcessOptions
\RequirePackage[T1]{fontenc}
\RequirePackage{geometry}
\RequirePackage{graphics}
\RequirePackage{ifpdf}
\RequirePackage[T1]{url}
\input{size1\@ptsize.clo}
\if@twoside
\geometry{a4paper,twoside,nohead,footskip=10mm,includefoot,
          left=25mm,right=25mm,top=30mm,bottom=12mm}
\else
\geometry{a4paper,nohead,footskip=10mm,includefoot,
          left=25mm,right=25mm,top=30mm,bottom=12mm}
\fi
\if@nadabrev@forcedcolor
\else
  \ifpdf
    \renewcommand{\@nadabrev@logocol}{cmyk}
  \else
    \renewcommand{\@nadabrev@logocol}{svv}
  \fi
\fi
\newcommand{\@nadabrev@default@logo}{kth_\@nadabrev@logocol\@nadabrev@logoname}

\setlength{\lineskip}{1pt}
\setlength{\normallineskip}{1pt}
\renewcommand\baselinestretch{}
\setlength{\parskip}{0.7em}
\setlength{\parindent}{0pt}
\setlength{\medskipamount}{\parskip}
\@lowpenalty   51
\@medpenalty  151
\@highpenalty 301
\setlength{\footnotesep}{12pt}
\setlength{\skip\footins}{10pt \@plus 2pt \@minus 4pt}
\def\ps@empty{%
  \let\@oddfoot\@empty\let\@oddhead\@empty
  \let\@evenfoot\@empty\let\@evenhead\@empty}
\def\ps@plain{%
  \let\@oddhead\@empty
  \def\@oddfoot{\if@nadabrev@foot
    \fontencoding{T1}\fontfamily{cmss}\selectfont
    \fontsize{7}{8.5}\selectfont
    \parbox{\textwidth}{\offinterlineskip
      \setlength{\lineskip}{4pt}
      \setlength{\normallineskip}{4pt}
      \begin{center}
        \makebox[\textwidth]{\hfill
          Kungliga Tekniska h\"ogskolan%
          \hfill
          \ifnum\@nameuse{@nadabrev@lastpage\the@nadabrev@letter}=1%
          \else
            \llap{\thepage
              \ifnum\@nameuse{@nadabrev@lastpage\the@nadabrev@letter}=0%
                \G@refundefinedtrue
              \else
                \space(\@nameuse{@nadabrev@lastpage\the@nadabrev@letter})%
              \fi
            }%
          \fi}\\
        \makebox[0pt]{\rule{180mm}{0.4pt}}\\
        \makebox[0pt]{\def\\{; }\strut
          \textbf{KTH} -- \fromaddress.
          Tel:~\telephonenum.
          Fax:~\@nadabrev@fax.
          E-post:~\@nadabrev@email}\\
        \makebox[0pt]{\strut
          \@nadabrev@accounts
          \settowidth{\@tempdima}{\@nadabrev@accounts}%
          \settowidth{\@tempdimb}{\@nadabrev@web}%
          \ifdim\@tempdima>1pt\ifdim\@tempdimb>1pt\relax
            \space
          \fi\fi
          \@nadabrev@web}
      \end{center}}\fi}
  \let\@evenfoot\@oddfoot}
\newcommand*{\name}[1]{\def\fromname{#1}}
\newsavebox{\@nadabrev@signature}
\newcounter{@nadabrev@signature}
\newlength{\@nadabrev@signaturewidth}
\newcommand*{\signature}[2][2]{%
  \setlength{\@nadabrev@signaturewidth}{\textwidth}%
  \divide \@nadabrev@signaturewidth by #1
  \def\and{\end{tabular}\end{minipage}%
    \addtocounter{@nadabrev@signature}{1}%
    \ifnum#1=\value{@nadabrev@signature}%
      \setcounter{@nadabrev@signature}{0}%
      \\*[5\medskipamount]%
    \fi
    \begin{minipage}[t]{\@nadabrev@signaturewidth}
      \begin{tabular}[t]{@{}l}}
  \begin{lrbox}{\@nadabrev@signature}\normalfont
    \begin{minipage}{\textwidth}
      \begin{minipage}[t]{\@nadabrev@signaturewidth}
        \begin{tabular}[t]{@{}l}
          #2
        \end{tabular}
      \end{minipage}
    \end{minipage}
  \end{lrbox}
}
\newcommand*{\accounts}[1]{\def\@nadabrev@accounts{#1}}
\newcommand*{\address}[2][\fromaddress]{%
  \def\fromaddress{#2}\def\@nadabrev@return{#1}}
\newcommand*{\location}[2][\fromlocation]{%
  \def\fromlocation{#2}\def\@nadabrev@loc{#1}}
\newcommand*{\telephone}[1]{\def\telephonenum{#1}}
\newcommand*{\telefax}[1]{\def\@nadabrev@fax{#1}}
\def\@nadabrev@url{\begingroup \urlstyle{same}\Url}
\def\email{\urldef{\@nadabrev@email}\@nadabrev@url}
\def\web{\urldef{\@nadabrev@web}\@nadabrev@url}
\address[\@nadabrev@loc\\SE-100 44~Stockholm\\Sweden]%
        {\fromlocation\\SE-100 44~Stockholm\\Sweden}
\location[KTH]{Valhallavägen 79}
\telephone{08~-~790~60~00}
\telefax{08~-~790~65~00}
\accounts{Pg:~1~56~53-9. Bg:~895-9223. Momsreg.nr/VAT:~SE202100305401.}
\date{Stockholm, \today}
\email{inst@nada.kth.se}
\web{www.kth.se}
\InputIfFileExists{kth-brev-site.cfg}
    {\typeout{Loading site specific letter configuration file.}}{}
\InputIfFileExists{kth-brev.cfg}
    {\typeout{Loading local letter configuration file.}}{}
\AtBeginDocument{%
  \@ifpackageloaded{babel}{\let\dateswedish=\datesymd}{}}
\newcommand*{\returnaddress}{\fromname\\KTH -- \@nadabrev@return}
\newcommand*{\makelabels}{%
  \AtBeginDocument{%
     \let\@startlabels\startlabels
     \let\@mlabel\mlabel
     \if@filesw
       \immediate\write\@mainaux{\string\@startlabels}\fi}%
  \AtEndDocument{%
     \if@filesw\immediate\write\@mainaux{\string\clearpage}\fi}}
\@onlypreamble\makelabels
\newcounter{@nadabrev@letter}
\def\@nadabrev@writelastpage{%
  \addtocounter{page}{-1}%
  \immediate\write\@auxout{%
    \string\global\string\@namedef
    {@nadabrev@lastpage\the@nadabrev@letter}{\thepage}}%
  \addtocounter{page}{1}%
  \addtocounter{@nadabrev@letter}{1}%
}
\AtEndDocument{\@nadabrev@writelastpage}
\newenvironment{letter}[2][\relax]{%
  \newpage
  \@nadabrev@writelastpage
  \@ifundefined{@nadabrev@lastpage\the@nadabrev@letter}%
  {\@namedef{@nadabrev@lastpage\the@nadabrev@letter}{0}}{}%
  \if@twoside
    \ifodd\c@page \else\thispagestyle{empty}\null\newpage\fi
  \fi
  \setcounter{page}{1}%
  \setcounter{footnote}{0}%
  \interlinepenalty=200 % smaller than the TeXbook value
  \def\@nadabrev@dnr{#1}%
  \@processto{\leavevmode\ignorespaces #2}%
}{%
  \stopletter\par\pagebreak\par
  \if@filesw
    \begingroup
    \let\\=\relax
    \let\protect\@unexpandable@protect
    \immediate\write\@auxout{%
      \string\@mlabel{\returnaddress}{\toname\\\toaddress}}%
    \endgroup
  \fi
}
\long\def\@processto#1{%
  \@xproc #1\\@@@%
  \ifx\toaddress\@empty
  \else
    \@yproc #1@@@%
  \fi}
\long\def\@xproc #1\\#2@@@{\def\toname{#1}\def\toaddress{#2}}
\long\def\@yproc #1\\#2@@@{\def\toaddress{#2}}
\newcommand*{\stopbreaks}{%
  \interlinepenalty\@M
  \def\par{\@@par\nobreak}%
  \let\\\@nobreakcr
  \let\vspace\@nobreakvspace}
\DeclareRobustCommand\@nobreakvspace
   {\@ifstar\@nobreakvspacex\@nobreakvspacex}
\def\@nobreakvspacex#1{%
  \ifvmode
    \nobreak\vskip #1\relax
  \else
    \@bsphack\vadjust{\nobreak\vskip #1}\@esphack
  \fi}
\def\@nobreakcr{\@ifstar{\@normalcr*}{\@normalcr*}}
\newcommand*{\startbreaks}{%
  \let\\\@normalcr
  \interlinepenalty 200%
  \def\par{\@@par\penalty 200\relax}}
\newcommand*{\@nadabrev@DIN}{%
  \put(5,-105){\line(1,0){2}}
  \put(5,-210){\line(1,0){2}}
  \put(148,-5){\line(0,-1){2}}
  \put( 20,-45){\line( 1,0){1}}  \put( 20,-45){\line(0,-1){1}}
  \put(105,-45){\line(-1,0){1}}  \put(105,-45){\line(0,-1){1}}
  \put( 20,-50){\line( 1,0){1}}
  \put(105,-50){\line(-1,0){1}}
  \put( 20,-90){\line( 1,0){1}}  \put( 20,-90){\line(0, 1){1}}
  \put(105,-90){\line(-1,0){1}}  \put(105,-90){\line(0, 1){1}}
  \put( 25,-50){\makebox(0,5)[l]{\tiny
      \parbox{105mm}{\def\\{; }\noindent
        \fromname\\KTH -- \@nadabrev@return}}}
  \put( 25,-90){\makebox(0,40)[l]{%
      \begin{tabular}{@{}l}\toname\\\toaddress\end{tabular}}}}
\newcommand*{\@nadabrev@KTH}{%
  \put(5,-105){\line(1,0){2}}
  \put(5,-210){\line(1,0){2}}
  \put(25,-80){\makebox(0,0)[lb]{%
      \begin{tabular}{@{}l}\toname\\\toaddress\end{tabular}}}}
\newcommand*{\@nadabrev@SS}{%
  \put(5,-105){\line(1,0){2}}
  \put(5,-210){\line(1,0){2}}
  \put(120,-64){\makebox(0,22)[l]{%
      \begin{tabular}{@{}l}\toname\\\toaddress\end{tabular}}}}
\newcommand*{\opening}[1]{%
  \centerline{%
    \setlength{\unitlength}{1mm}
    \begin{picture}(0,55)(105,-85)
      \if@nadabrev@head
        \put(105,-7){\makebox(0,0)[t]{%
            \scalebox{0.192}{\includegraphics{\@nadabrev@logo}}}}
      \fi
      \@nameuse{@nadabrev@\@nadabrev@layout}
      \put(120,-80){\makebox(0,0)[lb]{%
          \begin{tabular}{@{}l}\@nadabrev@dnr\end{tabular}}}
      \put(120,-85){\makebox(0,0)[lt]{%
          \begin{tabular}{@{}l}\@date\end{tabular}}}
    \end{picture}}
  \vspace*{5mm}
  #1\par\nobreak}
\newcommand{\closing}[1]{\par\nobreak\vspace{\parskip}%
  \stopbreaks
  \noindent #1\\[4\medskipamount]%
  \ifdim\wd\@nadabrev@signature=0pt
    \fromname
  \else
    \usebox{\@nadabrev@signature}%
  \fi
  \par}
\newcommand*{\cc}[1]{%
  \par\noindent
  \parbox[t]{\textwidth}{%
    \@hangfrom{\normalfont\ccname: }%
    \ignorespaces #1\strut}\par}
\newcommand*{\encl}[1]{%
  \par\noindent
  \parbox[t]{\textwidth}{%
    \@hangfrom{\normalfont\enclname: }%
    \ignorespaces #1\strut}\par}
\newcommand*{\ps}{\par\startbreaks}
\newcommand*{\stopletter}{}
\newcount\labelcount
\newcommand*{\startlabels}{\labelcount\z@
  \pagestyle{empty}%
  \let\@texttop\relax
  \topmargin -50\p@
  \headsep \z@
  \oddsidemargin -35\p@
  \evensidemargin -35\p@
  \textheight 10in
  \@colht\textheight  \@colroom\textheight \vsize\textheight
  \textwidth 550\p@
  \columnsep 26\p@
  \ifcase \@ptsize\relax
    \normalsize
  \or
    \small
  \or
    \footnotesize
  \fi
  \baselineskip \z@
  \lineskip \z@
  \boxmaxdepth \z@
  \parindent \z@
  \twocolumn\relax}
\let\@startlabels=\relax
\newcommand*{\mlabel}[2]{%
  \parbox[b][2in][c]{262\p@}{\strut\ignorespaces #2}%
  }
\let\@mlabel=\@gobbletwo
\setlength\leftmargini   {2.5em}
\setlength\leftmarginii  {2.2em}
\setlength\leftmarginiii {1.87em}
\setlength\leftmarginiv  {1.7em}
\setlength\leftmarginv   {1em}
\setlength\leftmarginvi  {1em}
\setlength\leftmargin    {\leftmargini}
\setlength  \labelsep  {5\p@}
\setlength  \labelwidth{\leftmargini}
\addtolength\labelwidth{-\labelsep}
\setlength\partopsep{0\p@}
\@beginparpenalty -\@lowpenalty
\@endparpenalty   -\@lowpenalty
\@itempenalty     -\@lowpenalty
\def\@listI{\setlength\leftmargin{\leftmargini}%
            \setlength\parsep {0\p@}%
            \setlength\topsep {.4em}%
            \setlength\itemsep{.4em}}
\let\@listi\@listI
\@listi
\def\@listii {\setlength  \leftmargin{\leftmarginii}%
              \setlength  \labelwidth{\leftmarginii}%
              \addtolength\labelwidth{-\labelsep}}
\def\@listiii{\setlength  \leftmargin{\leftmarginiii}%
              \setlength  \labelwidth{\leftmarginiii}%
              \addtolength\labelwidth{-\labelsep}%
              \setlength  \topsep    {.2em}%
              \setlength  \itemsep   {\topsep}}
\def\@listiv {\setlength  \leftmargin{\leftmarginiv}%
              \setlength  \labelwidth{\leftmarginiv}%
              \addtolength\labelwidth{-\labelsep}}
\def\@listv  {\setlength  \leftmargin{\leftmarginv}%
              \setlength  \labelwidth{\leftmarginv}%
              \addtolength\labelwidth{-\labelsep}}
\def\@listvi {\setlength  \leftmargin{\leftmarginvi}%
              \setlength  \labelwidth{\leftmarginvi}%
              \addtolength\labelwidth{-\labelsep}}
\renewcommand\theenumi{\@arabic\c@enumi}
\renewcommand\theenumii{\@alph\c@enumii}
\renewcommand\theenumiii{\@roman\c@enumiii}
\renewcommand\theenumiv{\@Alph\c@enumiv}
\newcommand\labelenumi{\theenumi.}
\newcommand\labelenumii{(\theenumii)}
\newcommand\labelenumiii{\theenumiii.}
\newcommand\labelenumiv{\theenumiv.}
\renewcommand\p@enumii{\theenumi}
\renewcommand\p@enumiii{\theenumi(\theenumii)}
\renewcommand\p@enumiv{\p@enumiii\theenumiii}
\newcommand\labelitemi{\textbullet}
\newcommand\labelitemii{\normalfont\bfseries \textendash}
\newcommand\labelitemiii{\textasteriskcentered}
\newcommand\labelitemiv{\textperiodcentered}
\newenvironment{description}
               {\list{}{\labelwidth\z@ \itemindent-\leftmargin
                        \let\makelabel\descriptionlabel}}
               {\endlist}
\newcommand*{\descriptionlabel}[1]{\hspace\labelsep
                                \normalfont\bfseries #1}
\newenvironment{verse}
               {\let\\=\@centercr
                \list{}{\setlength\itemsep{\z@}%
                        \setlength\itemindent{-15\p@}%
                        \setlength\listparindent{\itemindent}%
                        \setlength\rightmargin{\leftmargin}%
                        \addtolength\leftmargin{15\p@}}%
                \item[]}
               {\endlist}
\newenvironment{quotation}
               {\list{}{\setlength\listparindent{1.5em}%
                        \setlength\itemindent{\listparindent}%
                        \setlength\rightmargin{\leftmargin}}%
                \item[]}
               {\endlist}
\newenvironment{quote}
               {\list{}{\setlength\rightmargin{\leftmargin}}%
                \item[]}
               {\endlist}
\setlength\arraycolsep{5\p@}
\setlength\tabcolsep{6\p@}
\setlength\arrayrulewidth{.4\p@}
\setlength\doublerulesep{2\p@}
\setlength\tabbingsep{\labelsep}
\skip\@mpfootins = \skip\footins
\setlength\fboxsep{3\p@}
\setlength\fboxrule{.4\p@}
\renewcommand\theequation{\@arabic\c@equation}
\DeclareOldFontCommand{\rm}{\normalfont\rmfamily}{\mathrm}
\DeclareOldFontCommand{\sf}{\normalfont\sffamily}{\mathsf}
\DeclareOldFontCommand{\tt}{\normalfont\ttfamily}{\mathtt}
\DeclareOldFontCommand{\bf}{\normalfont\bfseries}{\mathbf}
\DeclareOldFontCommand{\it}{\normalfont\itshape}{\mathit}
\DeclareOldFontCommand{\sl}{\normalfont\slshape}{\relax}
\DeclareOldFontCommand{\sc}{\normalfont\scshape}{\relax}
\DeclareRobustCommand*{\cal}{\@fontswitch{\relax}{\mathcal}}
\DeclareRobustCommand*{\mit}{\@fontswitch{\relax}{\mathnormal}}
\renewcommand\footnoterule{%
  \kern-\p@
  \hrule \@width .4\columnwidth
  \kern .6\p@}
\long\def\@makefntext#1{%
    \noindent
    \hangindent 5\p@
    \hb@xt@5\p@{\hss\@makefnmark}#1}
\newcommand*{\ccname}{cc}
\newcommand*{\enclname}{encl}
\newcommand*{\pagename}{Page}
\newcommand*{\headtoname}{To}
\newcommand*{\today}{\ifcase\month\or
  January\or February\or March\or April\or May\or June\or
  July\or August\or September\or October\or November\or December\fi
  \space\number\day, \number\year}
\setlength\columnsep{10\p@}
\setlength\columnseprule{0\p@}
\pagestyle{plain}
\pagenumbering{arabic}
\raggedbottom
\onecolumn
\endinput
%%
%% End of file `kth-brev.cls'.
