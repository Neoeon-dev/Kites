#include "registermodel.h"
#include "common/register_names.h"
#include <QApplication>
#include <QBrush>
#include <QDebug>
#include <QPalette>

namespace Kites
{

RegisterModel::RegisterModel(QObject *parent, RegisterFile *regfile) : QAbstractTableModel(parent)
{
    m_currentRegisterFile = regfile;
    connect(m_currentRegisterFile, &RegisterFile::updateRegister, this,
            &RegisterModel::updateRegisterValue);
    connect(m_currentRegisterFile, &RegisterFile::updateFRegister, this,
            &RegisterModel::updateFRegisterValue);
    connect(m_currentRegisterFile, &RegisterFile::updateVRegister, this,
            &RegisterModel::updateVRegisterValue);
    connect(m_currentRegisterFile, &RegisterFile::vectorConfigChanged, this,
            &RegisterModel::registerResetSlot);
    connect(m_currentRegisterFile, &RegisterFile::registerResetSignal, this,
            &RegisterModel::registerResetSlot);
}

int RegisterModel::rowCount(const QModelIndex &parent) const
{
    Q_UNUSED(parent);
    return 96;
}

int RegisterModel::columnCount(const QModelIndex &parent) const
{
    Q_UNUSED(parent);
    return 3;
} // Register name, alias, value

void RegisterModel::setDisplayBase(Base base)
{
    beginResetModel();
    m_displayBase = base;
    endResetModel();
}

QVariant RegisterModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if (role != Qt::DisplayRole)
        return QVariant{};

    if (orientation == Qt::Horizontal)
    {
        switch (section)
        {
        case 0:
            return QString("Register");
        case 1:
            return QString("Alias");
        case 2:
            return QString("Value");
        }
    }

    return QVariant{};
}

QVariant RegisterModel::data(const QModelIndex &index, int role) const
{
    if (!m_currentRegisterFile)
        return QVariant{};

    if (role == Qt::TextAlignmentRole)
    {
        return Qt::AlignCenter;
    }
    if (role == Qt::BackgroundRole)
    {
        if (static_cast<size_t>(index.row()) == m_highlightedRegisterIndex)
        {
            return QBrush(QColor("#b13232fd")); // Highlight color for the updated register
        }
    }
    if (role == Qt::DisplayRole || role == Qt::ToolTipRole)
    {
        size_t row = static_cast<size_t>(index.row());
        switch (index.column())
        {
        case 0: // Register Name
            if (row < 32)
            {
                return QString("x%1").arg(row);
            }
            else if(row < 64)
            {
                return QString("f%1").arg(row - 32);
            }
            else if (row < 96)
            {
                return QString("v%1").arg(row - 64);
            }
            else{
                return QString("");
            }
        case 1: // Register Alias
        {
            if (row < 32)
                return QString(register_names::kGprAliases[row]);
            else if (row < 64)
                return QString(register_names::kFprAliases[row - 32]);
            else if (row < 96)
                return QString(register_names::kVregAliases[row - 64]);
            else
                return QString("");
        }
        case 2: // Register Value
        {
            if (row < 32)
            {
                uint64_t value = m_currentRegisterFile->ReadGpr(row);
                switch (m_displayBase)
                {
                case Base::Binary:
                    return QString("0b%1").arg(QString::number(value, 2));
                case Base::Decimal:
                    return QString::number(static_cast<int64_t>(value));
                case Base::Hexadecimal:
                    return QString("0x%1").arg(QString::number(value, 16).toUpper());
                }
            }
            else if (row < 64)
            {
                int fIndex = row - 32;

                // read raw bits of FP register (you must have this function)
                uint64_t raw = m_currentRegisterFile->ReadFpr(fIndex);

                switch (m_displayBase)
                {
                case Base::Binary:
                {
                    return QString("0b%1").arg(QString::number(raw, 2));
                }

                case Base::Decimal:
                {
                    uint32_t upper32 = static_cast<uint32_t>(raw >> 32);
                    if (upper32 == 0)
                    {
                        // Single-precision float (F-extension): value in lower 32 bits
                        uint32_t lower32 = static_cast<uint32_t>(raw & 0xFFFFFFFF);
                        float fval;
                        memcpy(&fval, &lower32, sizeof(float));
                        return QString::number(static_cast<double>(fval), 'g', 8);
                    }
                    else
                    {
                        // Double-precision (D-extension): full 64 bits
                        double dval;
                        memcpy(&dval, &raw, sizeof(double));
                        return QString::number(dval, 'g', 16);
                    }
                }

                case Base::Hexadecimal:
                {
                    return QString("0x%1").arg(QString::number(raw, 16).toUpper());
                }
                }
            }
            else if (row < 96)
            {
                const int vIndex = static_cast<int>(row) - 64;
                const size_t sewBits = currentSewBits();
                const QStringList elements = formatVectorElements(vIndex, sewBits);

                if (role == Qt::ToolTipRole)
                    return vectorTooltip(vIndex, sewBits, elements);

                // ", " separators let the view word-wrap the value onto several lines.
                return QString("[%1]").arg(elements.join(", "));
            }
            else
            {
                return QString("");
            } // TODO : add floating point register support
        }
        default:
            return QVariant{};
        }
    }
    return QVariant{};
}

size_t RegisterModel::currentSewBits() const
{
    // Element width comes from vtype.vsew. If vtype.vill is set (no valid
    // vsetvli yet) or vsew is reserved, fall back to 64-bit elements.
    const uint64_t vtype = m_currentRegisterFile->ReadCsr(0xC21);
    const unsigned vsew = (vtype >> 3) & 0b111;
    if (((vtype >> 63) & 1) || vsew > 3)
        return 64;
    return size_t{8} << vsew;
}

QStringList RegisterModel::formatVectorElements(int vIndex, size_t sewBits) const
{
    const size_t elementCount = architecture::VLEN_BITS / sewBits;
    QStringList elements;
    elements.reserve(static_cast<qsizetype>(elementCount));

    for (size_t i = 0; i < elementCount; ++i)
    {
        const uint64_t value = m_currentRegisterFile->ReadVectorElement(vIndex, i, sewBits);
        switch (m_displayBase)
        {
        case Base::Binary:
            elements.append(QString("0b%1").arg(QString::number(value, 2)));
            break;
        case Base::Decimal:
        {
            // Sign-extend from SEW bits so negative elements show as negative.
            int64_t signedValue = static_cast<int64_t>(value);
            if (sewBits < 64)
            {
                const unsigned shift = 64 - static_cast<unsigned>(sewBits);
                signedValue = static_cast<int64_t>(value << shift) >> shift;
            }
            elements.append(QString::number(signedValue));
            break;
        }
        case Base::Hexadecimal:
            elements.append(QString("0x%1").arg(QString::number(value, 16).toUpper()));
            break;
        }
    }
    return elements;
}

QString RegisterModel::vectorTooltip(int vIndex, size_t sewBits, const QStringList &elements) const
{
    // Rich-text grid: "[i] value" cells, with tail elements (i >= vl) greyed out.
    const uint64_t vl = m_currentRegisterFile->ReadCsr(0xC20);
    const int columns = sewBits >= 32 ? 4 : 8;

    QString html = QString("<b>v%1</b> &nbsp;·&nbsp; e%2 &nbsp;·&nbsp; vl = %3"
                           "<table cellspacing='0' cellpadding='2'>")
                       .arg(vIndex)
                       .arg(sewBits)
                       .arg(vl);

    for (int i = 0; i < elements.size(); ++i)
    {
        if (i % columns == 0)
            html += "<tr>";

        const bool tail = static_cast<uint64_t>(i) >= vl;
        html += QString("<td style='color:gray'>[%1]</td>"
                        "<td style='padding-right:10px;%2'><code>%3</code></td>")
                    .arg(i)
                    .arg(tail ? "color:gray" : "")
                    .arg(elements[i].toHtmlEscaped());

        if (i % columns == columns - 1)
            html += "</tr>";
    }
    html += "</table>";
    return html;
}

void RegisterModel::changeRegisterFile(RegisterFile *regfile)
{
    beginResetModel();
    m_currentRegisterFile = regfile;
    connect(m_currentRegisterFile, &RegisterFile::updateRegister, this,
            &RegisterModel::updateRegisterValue);
    connect(m_currentRegisterFile, &RegisterFile::updateFRegister, this,
            &RegisterModel::updateFRegisterValue);
    connect(m_currentRegisterFile, &RegisterFile::updateVRegister, this,
            &RegisterModel::updateVRegisterValue);
    connect(m_currentRegisterFile, &RegisterFile::vectorConfigChanged, this,
            &RegisterModel::registerResetSlot);
    // maybe we can directly connect it to
    connect(m_currentRegisterFile, &RegisterFile::registerResetSignal, this,
            &RegisterModel::registerResetSlot);
    endResetModel();
}

void RegisterModel::registerResetSlot()
{
    beginResetModel();
    endResetModel();
}

void RegisterModel::updateRegisterValue(size_t regIndex, uint64_t value)
{
    //QModelIndex index = this->index(static_cast<int>(regIndex), 2);
    m_highlightedRegisterIndex = regIndex; // Update highlighted register index
    // emit dataChanged(index,index);
    beginResetModel();
    endResetModel();
}

void RegisterModel::updateFRegisterValue(size_t regIndex, uint64_t value)
{
    //QModelIndex index = this->index(static_cast<int>(regIndex + 32), 2);
    m_highlightedRegisterIndex = regIndex + 32; // Update highlighted register index
    // emit dataChanged(index,index);
    beginResetModel();
    endResetModel();
}

void RegisterModel::updateVRegisterValue(size_t regIndex, VectorRegister value)
{
    m_highlightedRegisterIndex = regIndex + 64; // Update highlighted register index
    beginResetModel();
    endResetModel();
}
} // namespace Kites
