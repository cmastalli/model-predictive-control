%%%
%%% Class file used for final print of PhD thesis.
%%% Formats a G5 book with each G5 page centered on a corresponding A4 page.
%%%
%%% Written by Stefan Parkvall, August 27, 1996
%%%
%%%
\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{g5book}[1996/08/26 -- G5 book on A4 paper]
\LoadClass[a4paper,10pt]{book}
\RequirePackage{calc}
%%%
%%% Page size used for final print.
%%% Final version is to be printed with G5 size (239x169 mm),
%%% centered on A4 paper.
%%%
%%% (We could do without the 'calc' package, but the code would be
%%% less readable.)
%%%
\newlength{\gfivepaperwidth}
\setlength{\gfivepaperwidth}{169mm}
\newlength{\gfivepaperheight}
\setlength{\gfivepaperheight}{239mm}
\setlength{\textheight}{169mm}
\setlength{\textwidth}{111mm}
%%%
%%% Alter \voffset and \hoffset such that the G5 page will be centered
%%% on the A4 page.
%%%
\setlength{\hoffset}{(\paperwidth-\gfivepaperwidth)/2}
\setlength{\voffset}{(\paperheight-\gfivepaperheight)/2}
\addtolength{\voffset}{-1in}
\addtolength{\hoffset}{-1in}
%%%
%%% Set margins
%%%
\setlength{\oddsidemargin}%
  {(\gfivepaperwidth-\textwidth)*\real{.4}}
\setlength{\marginparwidth}%
  {(\gfivepaperwidth-\textwidth)*\real{.6}-\marginparsep-.4in}
\ifdim \marginparwidth >2in
  \setlength{\marginparwidth}{2in}
\fi
\setlength{\evensidemargin}{\gfivepaperwidth-\textwidth-\oddsidemargin}
\setlength{\topmargin}%
  {(\gfivepaperheight-\headheight-\headsep-\textheight-\footskip)/2}
%%%
%%%
%%%
