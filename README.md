### Cấu trúc báo cáo
- Chapter → Section → Subsection → Subsubsection → Dot → '-' → '+'

### Template cho hình ảnh:
```latex
\begin{figure}[H]
    \centering
    \includegraphics[width=1\textwidth]{pictures/image.png}
    \caption{Mô tả hình ảnh}
    \label{fig:label}
\end{figure}
```

### Template cho bảng:
```latex
\begin{table}[H]
    \centering
    \begin{tabular}{|c|c|c|}
        \hline
        \textbf{Cột 1} & \textbf{Cột 2} & \textbf{Cột 3} \\
        \hline
        Dữ liệu 1 & Dữ liệu 2 & Dữ liệu 3 \\
        \hline
    \end{tabular}
    \caption{Mô tả bảng}
    \label{tab:label}
\end{table}
```